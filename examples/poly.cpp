#include <synthlab.hpp>

struct VoiceData {
  sl::Env env;
  float pb; // freq multiplier
  VoiceData() : pb(1.0) {
    env.add(sl::Env::Set(0));
    env.add(sl::Env::Slide(1.0,0.2));
    env.add(sl::Env::Slide(0.7,0.1));
    env.add(sl::Env::Sustain());
    env.add(sl::Env::Slide(0.0,1.0));
  }
  void cc(int num, int value) {
    std::cout << "num=" << num << ", value=" << value << "\n";
    switch (num) {
    case 1:
      // attack time
      env[1].setTime((5*value)/127.0);
      break;
    case 2:
      // decay time
      env[2].setTime(value/127.0);
      break;
    case 3:
      // sustain level
      env[2].setTarget(value/127.0);
      break;
    case 4:
      // release time
      env[4].setTime((5*value)/127.0);
      break;
    case 128:
      // pitch bend
      pb = pow(2,((float)value-0x2000)/8192.0);
      break;
    }
  }
};

class Voice : public sl::Generator<0,1> {
  sl::SineOsc osc_;
  sl::Env env_;
  float gain_;
  float basefreq_;
  float pb_;
public:
  Voice() :
    gain_(0), basefreq_(220), pb_(1)
  {
  }
  bool render(const VoiceData& voicedata, int nframes, InputTrunk &input, OutputTrunk &output) {
    if (pb_ != voicedata.pb) {
      pb_ = voicedata.pb;
      osc_.setFreq(basefreq_ * pb_);
    }
    osc_.render(nframes, output);
    output.mul(nframes, gain_);
    OutputTrunk env_output;
    bool playing = env_.render(nframes, env_output);
    output.mul(nframes, env_output);
    return playing;
  }
  void play(const VoiceData& voicedata, int midiNote, int midiVel) {
    // each voice gets its own copy of the shared env
    //
    // => control changes don't affect voices already playing
    env_ = voicedata.env;
    basefreq_ = sl::midi2cps(midiNote);
    pb_ = voicedata.pb;
    osc_.play(basefreq_ * pb_);
    env_.play();
    gain_ = midiVel / 127.0;
  }
  void release(int delay) {
    env_.release(delay);
  }
};

int main(int argc, char **argv) {
  sl::PolySynth<Voice, VoiceData> synth("poly");
  synth.start();
  getchar();
}
