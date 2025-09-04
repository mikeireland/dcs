"""
Autoalign the heimdallr beams using the c red one data stream and the MDS
Uses only K1, since it is brighter

The overall structure is:
1. shutter all beams off except 1, pause for a short time
2. find the centre of the blob in the image
3. save the pixel offsets
4. repeat for all beams
5. unshutter all beams
6. move them using the offsets + moveimage like calculation
"""

from xaosim.shmlib import shm
import numpy as np
import matplotlib.pyplot as plt
import time
import asgard_alignment.Engineering as asgE
import zmq
import os
from tqdm import tqdm
import scipy.ndimage as ndi

from scipy.optimize import curve_fit
import argparse


class HeimdallrAA:
    def __init__(self, shutter_pause_time, band, flux_threshold):
        # Set target_pixels and col_bnds based on band
        # TODO: read values from config file
        if band.upper() == "K1":
            self.target_pixels = (27, 49)
            self.col_bnds = (0, 160)
        elif band.upper() == "K2":
            self.target_pixels = (289, 51)
            self.col_bnds = (160, 320)
        else:
            raise ValueError("Unknown band: choose 'K1' or 'K2'.")

        self.flux_threshold = flux_threshold

        self.mds = self._open_mds_connection()

        self.stream = self._open_stream_connection()

        self._shutter_pause_time = (
            shutter_pause_time  # seconds to pause after shuttering
        )

        self.row_bnds = (0, 128)
        # self.col_bnds is set above

    # MDS interface
    def _open_mds_connection(self):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 10000)
        server_address = "tcp://192.168.100.2:5555"
        socket.connect(server_address)
        return socket

    def _send_and_get_response(self, message):
        # print("sending", message)
        self.mds.send_string(message)
        response = self.mds.recv_string()
        # print("response", response)
        return response.strip()

    # stream interface
    def _open_stream_connection(self):
        stream_path = "/dev/shm/cred1.im.shm"
        if not os.path.exists(stream_path):
            raise FileNotFoundError(f"Stream file {stream_path} does not exist.")
        return shm(stream_path)

    # processing
    def _find_blob_centre(self, frame):
        # use ndi median filter and then a gaussian filter on cropped image
        cropped_frame = frame[
            self.row_bnds[0] : self.row_bnds[1], self.col_bnds[0] : self.col_bnds[1]
        ]

        filtered_frame = ndi.median_filter(cropped_frame, size=3)
        filtered_frame = ndi.gaussian_filter(filtered_frame, sigma=2)

        max_loc_cropped = np.unravel_index(
            np.argmax(filtered_frame), filtered_frame.shape
        )
        max_loc_cropped = np.array(max_loc_cropped)
        max_loc = max_loc_cropped + np.array([self.row_bnds[0], self.col_bnds[0]])

        return max_loc

    def _get_frame(self):
        full_frame = self.stream.get_data().mean(0)
        return full_frame

    def _get_and_process_blob(self):
        full_frame = self._get_frame() - 1000.0
        blob_centre = self._find_blob_centre(full_frame)
        return blob_centre

    def _get_blob_with_flux(self, radius):
        full_frame = self._get_frame() - 1000.0
        blob_centre = self._find_blob_centre(full_frame)
        mask = self.circle_mask(full_frame.shape, blob_centre, radius)
        blob_flux = full_frame[mask].sum()
        return blob_centre, blob_flux

    @staticmethod
    def circle_mask(shape, center, radius):
        y, x = np.ogrid[: shape[0], : shape[1]]
        dist_from_center = np.sqrt((x - center[1]) ** 2 + (y - center[0]) ** 2)
        mask = dist_from_center <= radius
        return mask

    def autoalign_coarse_parallel(self):
        # 1. shutter all beams off except 1, pause for a short time
        # 2. find the centre of the blob in the image
        # 3. save the pixel offsets
        # 4. repeat for all beams
        pixel_offsets = {}

        msg = f"h_shut close 2"
        self._send_and_get_response(msg)
        msg = f"h_shut close 3"
        self._send_and_get_response(msg)
        msg = f"h_shut close 4"
        self._send_and_get_response(msg)
        msg = f"h_shut open 1"
        self._send_and_get_response(msg)
        time.sleep(self._shutter_pause_time)

        flux_radius = 6

        for target_beam in range(1, 5):
            print(f"doing beam {target_beam}")
            if target_beam > 1:
                msg = f"h_shut close {target_beam-1}"
                self._send_and_get_response(msg)
                msg = f"h_shut open {target_beam}"
                self._send_and_get_response(msg)
                time.sleep(self._shutter_pause_time)

            blob_centre, flux = self._get_blob_with_flux(flux_radius)

            if flux < self.flux_threshold:
                print(
                    f"Beam {target_beam} has low flux: {flux}. setting delta to zero."
                )
                pixel_offsets[target_beam] = np.array([0, 0])
            else:
                print(f"Beam {target_beam} flux: {flux:.2f}, centre: {blob_centre}")
                # calculate the pixel offsets from the target pixels
                pixel_offsets[target_beam] = np.array(
                    [
                        self.target_pixels[1] - blob_centre[0],
                        self.target_pixels[0] - blob_centre[1],
                    ]
                )

        # 5. unshutter all beams
        msg = "h_shut open 1,2,3,4"
        self._send_and_get_response(msg)
        time.sleep(self._shutter_pause_time)

        # 6. move them using the offsets + moveimage like calculation
        # key here is to parallelise
        uv_commands = {}
        for beam, offset in pixel_offsets.items():
            uv_cmd = asgE.move_img_calc("c_red_one_focus", beam, offset)
            uv_commands[beam] = uv_cmd

        # send commands
        axis_list = ["HTPP", "HTTP", "HTPI", "HTTI"]
        axes = [
            [axis + str(beam_number) for axis in axis_list]
            for beam_number in range(1, 5)
        ]

        for beam, uv_cmd in uv_commands.items():
            cmd = f"moverel {axes[beam-1][0]} {uv_cmd[0]}"
            self._send_and_get_response(cmd)
            cmd = f"moverel {axes[beam-1][2]} {uv_cmd[2]}"
            self._send_and_get_response(cmd)

        time.sleep(0.4)

        for beam, uv_cmd in uv_commands.items():
            cmd = f"moverel {axes[beam-1][1]} {uv_cmd[1]}"
            self._send_and_get_response(cmd)
            cmd = f"moverel {axes[beam-1][3]} {uv_cmd[3]}"
            self._send_and_get_response(cmd)

    def autoalign_3_pupil(self):
        mv_time = 2.5

        beam = 3
        msg = f"h_shut close 1,2,4"
        self._send_and_get_response(msg)
        msg = f"h_shut open 3"
        self._send_and_get_response(msg)
        time.sleep(self._shutter_pause_time)

        # 2. rough align beam 3
        blob_centre = self._get_and_process_blob()

        pix_offset = np.array(
            [
                self.target_pixels[1] - blob_centre[0],
                self.target_pixels[0] - blob_centre[1],
            ]
        )
        uv_cmd = asgE.move_img_calc("c_red_one_focus", beam, pix_offset)

        axis_list = ["HTPP", "HTTP", "HTPI", "HTTI"]
        axes = [axis + str(beam) for axis in axis_list]

        cmd = f"moverel {axes[0]} {uv_cmd[0]}"
        self._send_and_get_response(cmd)
        cmd = f"moverel {axes[2]} {uv_cmd[2]}"
        self._send_and_get_response(cmd)
        time.sleep(0.5)
        cmd = f"moverel {axes[1]} {uv_cmd[1]}"
        self._send_and_get_response(cmd)
        cmd = f"moverel {axes[3]} {uv_cmd[3]}"
        self._send_and_get_response(cmd)
        time.sleep(0.5)

        # 3. move pupil to optimize flux
        pup_offset = 0.2  # mm
        n_samp = 7
        flux_beam_radius = 6  # pixels

        measurement_locs_x = np.linspace(-pup_offset, pup_offset, n_samp)
        relative_measurement_locs = np.array(
            [
                measurement_locs_x[i] - measurement_locs_x[i - 1]
                for i in range(1, n_samp)
            ]
        )
        relative_measurement_locs = np.concatenate(
            [[-pup_offset], relative_measurement_locs]
        )

        def fit_func(x, m, a, b, c):
            return -m * np.abs(x - a) - m * np.abs(x - b) + c

        def get_peak_flux_loc(pos, fluxes):
            fluxes /= np.max(fluxes)  # normalise
            params, _ = curve_fit(fit_func, pos, fluxes, p0=[1, -0.05, 0.05, 1])

            return (params[1] + params[2]) / 2

        fluxes = []
        for delta in relative_measurement_locs:
            cmd = f"mv_pup c_red_one_focus {beam} {delta} {0.0}"
            self._send_and_get_response(cmd)
            print("sent", cmd)
            time.sleep(mv_time)
            _, flux = self._get_blob_with_flux(flux_beam_radius)
            fluxes.append(flux)

        cur_pupil_pos = measurement_locs_x[-1]

        fluxes = np.array(fluxes)
        if np.max(fluxes) < self.flux_threshold:
            print(
                f"Beam {beam} has low flux: {np.max(fluxes)}. Setting pupil offset back to original."
            )
            del_needed = -cur_pupil_pos
            cmd = f"mv_pup c_red_one_focus {beam} {del_needed} {0.0}"
            self._send_and_get_response(cmd)
            return

        optimal_offset = get_peak_flux_loc(measurement_locs_x, fluxes)

        del_needed = optimal_offset - cur_pupil_pos

        print(
            f"Optimal pupil offset for beam {beam} x: {optimal_offset:.4f} mm ({del_needed:.4f} mm from current position)"
        )
        cmd = f"mv_pup c_red_one_focus {beam} {del_needed} {0.0}"
        self._send_and_get_response(cmd)

        time.sleep(2)

        fluxes = []
        for delta in relative_measurement_locs:
            cmd = f"mv_pup c_red_one_focus {beam} {0.0} {delta}"
            self._send_and_get_response(cmd)
            print("sent", cmd)
            time.sleep(2.5)
            _, flux = self._get_blob_with_flux(flux_beam_radius)
            fluxes.append(flux)

        fluxes = np.array(fluxes)

        optimal_offset = get_peak_flux_loc(measurement_locs_x, fluxes)
        cur_pupil_pos = measurement_locs_x[-1]

        del_needed = optimal_offset - cur_pupil_pos

        print(
            f"Optimal pupil offset for beam {beam} y: {optimal_offset:.4f} mm ({del_needed:.4f} mm from current position)"
        )
        cmd = f"mv_pup c_red_one_focus {beam} {0.0} {del_needed}"
        self._send_and_get_response(cmd)

        # open all shutters
        msg = f"h_shut open 1,2,3,4"
        self._send_and_get_response(msg)

    def autoalign_full(self):
        pass


def main():

    parser = argparse.ArgumentParser(description="Autoalign Heimdallr beams.")
    parser.add_argument(
        "--shutter_pause_time",
        type=float,
        default=2.5,
        help="Seconds to pause after shuttering (default: 2.5)",
    )
    parser.add_argument(
        "-a",
        "--align",
        type=str,
        required=True,
        choices=["cp", "coarseparallel", "p3", "pupil3"],
        help="Alignment method: 'cp'/'coarseparallel' or 'p3'/'pupil3'",
    )
    parser.add_argument(
        "-b",
        "--band",
        type=str,
        default="K2",
        choices=["K1", "K2"],
        help="Band to use: 'K1' or 'K2' (default: K2)",
    )
    args = parser.parse_args()

    heimdallr_aa = HeimdallrAA(
        shutter_pause_time=args.shutter_pause_time,
        band=args.band,
        flux_threshold=200.0,
    )

    if args.align in ["cp", "coarseparallel"]:
        heimdallr_aa.autoalign_coarse_parallel()
    elif args.align in ["p3", "pupil3"]:
        heimdallr_aa.autoalign_3_pupil()
    else:
        raise ValueError("Unknown alignment method.")

    print("Autoalignment completed.")


if __name__ == "__main__":
    main()
