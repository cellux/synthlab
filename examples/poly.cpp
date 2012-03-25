#include <synthlab.h>

class Voice : public sl::Generator<0,1> {
  sl::SineOsc osc_;
  sl::Env env_;
  float gain_;
  bool playing_;
public:
  Voice() :
    gain_(0),
    playing_(false)
  {
    env_.add(sl::Env::Set(0));
    env_.add(sl::Env::Slide(1.0,0.2));
    env_.add(sl::Env::Slide(0.7,0.1));
    env_.add(sl::Env::Sustain());
    env_.add(sl::Env::Slide(0.0,1.0));
  }
  bool render(sl::SampleCount nframes, InputTrunk &input, OutputTrunk &output) {
    osc_.render(nframes, output);
    output.mul(nframes, gain_);
    OutputTrunk env_output;
    playing_ = env_.render(nframes, env_output);
    output.mul(nframes, env_output);
    return playing_;
  }
  void play(unsigned char midiNote, unsigned char midiVel) {
    osc_.play(sl::midi2cps(midiNote));
    env_.play();
    gain_ = midiVel / 127.0;
    playing_ = true;
  }
  void release(sl::SampleCount delay) {
    env_.release(delay);
  }
};

int main(int argc, char **argv) {
  sl::PolySynth<Voice> synth("poly");
  synth.start();
  while (true) {
    sleep(1);
  }
}
