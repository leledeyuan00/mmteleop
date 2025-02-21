#ifndef LOW_PASS_FILTER_HPP__
#define LOW_PASS_FILTER_HPP__


// low pass filter
#include <control_toolbox/control_toolbox/filters.hpp>
#include <Eigen/Dense>
#include <memory>

namespace garment_research{

class LowPassFilter
{
public:
    LowPassFilter(Eigen::Vector3d alpha)
    {
        alpha_ = alpha;
        first_time_ = true;
    }

    Eigen::Vector3d update(Eigen::Vector3d input)
    {
        if (first_time_)
        {
            first_time_ = false;
            output_ = input;
        }
        else
        {
            for (size_t i = 0; i < 3; i++)
            {
                output_(i) = filters::exponentialSmoothing(input(i), output_(i), alpha_(i));
            }            
        }
        return output_;
    }

private:
    Eigen::Vector3d alpha_;
    Eigen::Vector3d output_;
    bool first_time_;
}; // class LowPassFilter

} // namespace garment_research

#endif // LOW_PASS_FILTER_HPP__