#ifndef SYNTHLAB_JACKAUDIOPROVIDER_HPP
#define SYNTHLAB_JACKAUDIOPROVIDER_HPP

namespace sl {

  template <class Synth>
  class JackAudioProvider {
    jack_client_t *client_;
    jack_port_t* midiInPort_;
    jack_port_t* inputPorts_[Synth::InputBuffer::nChannels];
    jack_port_t* outputPorts_[Synth::OutputBuffer::nChannels];
    Synth* synth_;
    unsigned char midiCtrlMap_[128];

    static int jackSampleRateCallback(jack_nframes_t sr, void *arg) {
      sl::sampleRate(sr);
      return 0;
    }

    static int jackBufferSizeCallback(jack_nframes_t bs, void *arg) {
      sl::bufferSize(bs);
      return 0;
    }

    static int jackProcessCallback(jack_nframes_t nframes, void *arg) {
      JackAudioProvider *jap = static_cast<JackAudioProvider*>(arg);
      Synth *synth = jap->synth_;
      SampleBufferAllocator::reset();
      typename Synth::InputBuffer input;
      typename Synth::OutputBuffer output;
      for (int i=0; i<Synth::InputBuffer::nChannels; i++) {
	jack_default_audio_sample_t *in = (jack_default_audio_sample_t*) jack_port_get_buffer(jap->inputPorts_[i], nframes);
	Sample *buf = input[i];
	std::copy(in,in+nframes,buf);
      }
      void *midiBuf = jack_port_get_buffer(jap->midiInPort_, nframes);
      jack_nframes_t midiEventCount = jack_midi_get_event_count(midiBuf);
      jack_midi_event_t midiEvent;
      for (int i=0; i<midiEventCount; i++) {
	jack_midi_event_get(&midiEvent, midiBuf, i);
	unsigned char midiCommand = *(midiEvent.buffer) & 0xf0;
        //std::cout << "got midi command: " << int(midiCommand) << "\n";
	if (midiEvent.size > 2) {
	  unsigned char midiValue1 = *(midiEvent.buffer+1);
	  unsigned char midiValue2 = *(midiEvent.buffer+2);
	  int delay = midiEvent.time;
	  switch (midiCommand) {
	  case 0x80:
	    synth->noteOff(midiValue1, midiValue2, delay);
	    break;
	  case 0x90:
	    synth->noteOn(midiValue1, midiValue2, delay);
	    break;
          case 0xb0:
            synth->controlChange(jap->midiCtrlMap_[midiValue1], midiValue2);
            break;
          case 0xe0:
            // pitch bend
            synth->controlChange(128, midiValue1 | (midiValue2<<7));
            break;
	  }
	}
      }
      synth->render(nframes, output, input);
      for (int i=0; i<Synth::OutputBuffer::nChannels; i++) {
	jack_default_audio_sample_t *out = (jack_default_audio_sample_t*) jack_port_get_buffer(jap->outputPorts_[i], nframes);
	Sample *buf = output[i];
	std::copy(buf,buf+nframes,out);
      }
      return 0;
    }

  public:
    // exceptions
    class JackClientOpenError {};
    class JackActivateError {};
    class JackDeactivateError {};

    JackAudioProvider(Synth *synth, const std::string &name)
      : synth_(synth)
    {
      SampleBufferAllocator::reset();
      client_ = jack_client_open(name.c_str(), JackNoStartServer, 0);
      if (client_ == NULL) {
	throw JackClientOpenError();
      }
      std::cout << "connected to jackd as client `" << name << "`\n";
      sl::sampleRate(jack_get_sample_rate(client_));
      jack_set_sample_rate_callback(client_, jackSampleRateCallback, 0);
      sl::bufferSize(jack_get_buffer_size(client_));
      jack_set_buffer_size_callback(client_, jackBufferSizeCallback, 0);
      jack_set_process_callback(client_, jackProcessCallback, this);
      midiInPort_ = jack_port_register(client_, "midi_in", JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0);
      std::cout << "registered midi port: midi_in\n";
      char portName[64];
      for (int i=0; i<Synth::InputBuffer::nChannels; i++) {
	snprintf(portName, sizeof(portName), "in_%u", i+1);
	inputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
        std::cout << "registered audio port: " << portName << "\n";
      }
      for (int i=0; i<Synth::OutputBuffer::nChannels; i++) {
	snprintf(portName, sizeof(portName), "out_%u", i+1);
	outputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
        std::cout << "registered audio port: " << portName << "\n";
      }
      for (int i=0; i<128; i++) {
        midiCtrlMap_[i] = i;
      }
      for (int i=11; i<=19; i++) {
        midiCtrlMap_[i] = i-10;
      }
      midiCtrlMap_[1] = 129; // modwheel
    }

    ~JackAudioProvider() {
      if (client_ != 0) {
	jack_client_close(client_);
      }
    }

    void start() {
      if (jack_activate(client_) != 0) {
	throw JackActivateError();
      }
    }

    void stop() {
      if (jack_deactivate(client_) != 0) {
	throw JackDeactivateError();
      }
    }
  };

}

#endif
