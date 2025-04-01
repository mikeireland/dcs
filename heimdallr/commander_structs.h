//Should be auto-created by parsing heimdallr.h
namespace nlohmann {
    template <>
    struct adl_serializer<EncodedImage> {
        static void to_json(json& j, const EncodedImage& p) {
            j = json{{"szx", p.szx}, {"szy", p.szy}, {"type", p.type}, {"message", p.message}};
        }
        static void from_json(const json& j, EncodedImage& p) {
            j.at("szx").get_to(p.szx);
            j.at("szy").get_to(p.szy);
            j.at("type").get_to(p.type);
            j.at("message").get_to(p.message);

        }
    };
}