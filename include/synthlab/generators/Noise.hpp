#ifndef SYNTHLAB_NOISE_HPP
#define SYNTHLAB_NOISE_HPP

namespace sl {

  class Noise : public Gen<0,1> {
  public:
    void render(int nframes, Sample *output) {
      for (int i=0; i<nframes; i++) {
	*output++ = (random()*2.0/RAND_MAX)-1.0;
      }
    }
  };

}

#endif
