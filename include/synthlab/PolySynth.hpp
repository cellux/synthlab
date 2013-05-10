#ifndef SYNTHLAB_POLYSYNTH_HPP
#define SYNTHLAB_POLYSYNTH_HPP

namespace sl {

  template <class Voice,
	    int MAXVOICES=16,
	    template <class Synth> class AudioProvider = JackAudioProvider>
  class PolySynth {
  public:
    typedef typename Voice::InputBuffer InputBuffer;
    typedef typename Voice::OutputBuffer OutputBuffer;
    typedef typename Voice::Data VoiceData;
    PolySynth(const std::string &name)
      : audioProvider_(this, name)
    {
      for (int i=0; i<MAXVOICES; i++) {
	voices_[i].data(&voiceData_);
      }
    }
    ~PolySynth() {
      stop();
    }
    void render(int nframes, OutputBuffer &output, InputBuffer &input) {
      voiceData_.render(nframes);
      output.fill(nframes, 0);
      OutputBuffer o;
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].status != VS_INACTIVE) {
	  int delay = voiceInfo_[i].delay;
	  bool voiceStillActive = voices_[i].render(nframes-delay, o, input);
          if (! voiceStillActive) {
            voiceInfo_[i].status = VS_INACTIVE;
          }
	  output.add(nframes-delay, o, delay); // nframes, input, offset
	  voiceInfo_[i].delay = 0;
	}
      }
    }
    void noteOn(int midiNote, int midiVel, int delay) {
      int inactive = -1;
      for (int i=0; i<MAXVOICES; i++) {
        if (inactive == -1 && voiceInfo_[i].status == VS_INACTIVE) inactive = i;
        if (voiceInfo_[i].midiNote == midiNote && voiceInfo_[i].status == VS_PLAYING) {
          voices_[i].release(delay);
          voiceInfo_[i].status = VS_RELEASING;
        }
      }
      if (inactive >= 0) {
	voices_[inactive].play(midiNote, midiVel);
	voiceInfo_[inactive].midiNote = midiNote;
	voiceInfo_[inactive].midiVel = midiVel;
	voiceInfo_[inactive].delay = delay;
	voiceInfo_[inactive].status = VS_PLAYING;
      }
    }
    void noteOff(int midiNote, int midiVel, int delay) {
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].status != VS_PLAYING) continue;
	if (voiceInfo_[i].midiNote != midiNote) continue;
	voices_[i].release(delay);
        voiceInfo_[i].status = VS_RELEASING;
	break;
      }
    }
    void controlChange(int num, int value) {
      voiceData_.cc(num, value);
    }
    void start() {
      audioProvider_.start();
    }
    void stop() {
      audioProvider_.stop();
    }

  private:
    AudioProvider<PolySynth> audioProvider_;
    Voice voices_[MAXVOICES];
    VoiceInfo voiceInfo_[MAXVOICES];
    VoiceData voiceData_;
  };

}

#endif
