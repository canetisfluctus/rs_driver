
#include "rs_lidar/decoder/decoder_16.hpp"
#include "rs_lidar/decoder/decoder_32.hpp"
#include "rs_lidar/decoder/decoder_128.hpp"
#include "rs_lidar/decoder/decoder_bp.hpp"
namespace robosense
{
namespace sensor
{
template <typename vpoint>
class DecoderFactory
{

public:
    inline static std::shared_ptr<DecoderBase<vpoint>> createDecoder(const std::string &lidar_type, ST_Param param)
    {
        if (lidar_type == "RS16")
        {
            return std::make_shared<Decoder16<vpoint>>(param);
        }
        else if (lidar_type == "RS32")
        {
            return std::make_shared<Decoder32<vpoint>>(param);
        }
        else if (lidar_type == "RSBP")
        {
            return std::make_shared<DecoderBP<vpoint>>(param);
        }
        else if (lidar_type == "RS128")
        {
            return std::make_shared<Decoder128<vpoint>>(param);
        }
        else
        {
            ERROR<<"Wrong lidar type : "<<lidar_type<<REND;
            ERROR<<"Please set RS16 or RS32 or RSBP or RS128 ! "<<REND;
            exit(-1);
        }
    }
};

} // namespace sensor
} // namespace robosense
