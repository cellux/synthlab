#ifndef SYNTHLAB_SLIDINGVALUE_HPP
#define SYNTHLAB_SLIDINGVALUE_HPP

namespace sl {

  class SlidingValue : public Gen<0,1> {
    Sample value_;
    Sample increment_;
    int length_;
    int slideLength_;
  public:
    SlidingValue(Sample value, int slideLength=sl::bufferSize())
      : value_(value),
        increment_(0),
        length_(0),
        slideLength_(slideLength)
    {}
    bool render(int nframes, Sample *output) {
      if (length_ == 0) {
        std::fill_n(output, nframes, value_);
      }
      else if (length_ >= nframes) {
        for (int i=0; i<nframes; i++) {
          output[i] = value_;
          value_ += increment_;
        }
        length_ -= nframes;
      }
      else {
        for (int i=0; i<length_; i++) {
          output[i] = value_;
          value_ += increment_;
        }
        length_ = 0;
      }
      return true;
    }
    void setTarget(Sample target) {
      length_ = slideLength_;
      increment_ = (target - value_) / length_;
    }
    void setSlideLength(int slideLength) {
      slideLength_ = slideLength;
    }
    Sample value() { return value_; }
  };

}

#endif
