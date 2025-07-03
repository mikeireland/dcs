import telnetlib

HOST = "192.168.100.11"
user = "teladmin"
pwd = "asgard"

tn = telnetlib.Telnet(HOST, 23)

# print("OK0")
# tn.read_until(b"login: ")
# print("OK1")
# tn.write(user.encode('ascii') + b"\n")
# print("OK2")
# tn.read_until(b"Password: ")
# print("OK3")
# tn.write(pwd.encode('ascii') + b"\n")
# print("OK4")
# tn.write(b"read meter olt o04 pow" + b"\n")


# print(tn.read_lazy().decode('ascii'))
