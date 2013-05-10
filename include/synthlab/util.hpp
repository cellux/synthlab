#ifndef SYNTHLAB_UTIL_HPP
#define SYNTHLAB_UTIL_HPP

namespace sl {

  template <class T> struct AbsCmp : std::binary_function<T,T,T> {
    T operator() (const T &x, const T &y) const {
      return (fabs(x) < fabs(y));
    }
  };

  template <class Iter>
  Sample peakAmplitude(Iter first, Iter last) {
    typedef typename std::iterator_traits<Iter>::value_type value_type;
    return fabs(*std::max_element(first, last, AbsCmp<value_type>()));
  }

  template <class T> struct MeanPowerAcc : std::binary_function<T,T,T> {
    T operator() (const T &x, const T &y) const {
      return x+y*y;
    }
  };

  template <class Iter>
  Sample meanPower(Iter first, Iter last) {
    typedef typename std::iterator_traits<Iter>::value_type value_type;
    if (first == last) {
      return 0;
    }
    else {
      return std::accumulate(first, last, 0, MeanPowerAcc<value_type>())/(last-first);
    }
  }

  template <class Iter>
  Sample rmsAmplitude(Iter first, Iter last) {
    return sqrt(meanPower(first, last));
  }

  const float dbRefAmplitude = 0.00001;

  float amp2db(float amp) {
    return 20*log10(amp/dbRefAmplitude);
  }

  float midi2cps(unsigned char midiNote) {
    return 440.0 * pow(2.0, (midiNote-69)/12.0);
  }

  unsigned char cps2midi(float cps) {
    return 69 + 12*log2(cps/440);
  }

}

#endif
