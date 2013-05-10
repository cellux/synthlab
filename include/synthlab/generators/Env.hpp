#ifndef SYNTHLAB_ENV_HPP
#define SYNTHLAB_ENV_HPP

namespace sl {

  class Env : public Gen<0,1> {
    enum EnvCommandType {
      SET,
      SLIDE,
      SUSTAIN
    };
    struct EnvCommand {
      EnvCommandType type;
      union {
	struct {
	  Sample value;
	} set;
	struct {
	  Sample target;
	  float time;
	} slide;
      };
      void setValue(Sample value) {
        set.value = value;
      }
      void setTarget(Sample target) {
        slide.target = target;
      }
      void setTime(float time) {
        slide.time = time;
      }
    };
    typedef std::vector<EnvCommand> EnvCommandVec;
    EnvCommandVec commands_;
    EnvCommandVec::const_iterator ip_;
    Sample value_;
    Sample increment_;
    Sample length_;
  public:
    class BadEnvCommandType {};

    Env()
      : commands_(),
	ip_(commands_.begin()),
	value_(0),
	increment_(0),
	length_(0)
    {
    }
    static EnvCommand Set(Sample value) {
      EnvCommand cmd;
      cmd.type = SET;
      cmd.set.value = value;
      return cmd;
    }
    static EnvCommand Slide(Sample target, float time) {
      EnvCommand cmd;
      cmd.type = SLIDE;
      cmd.slide.target = target;
      cmd.slide.time = time;
      return cmd;
    }
    static EnvCommand Sustain() {
      EnvCommand cmd;
      cmd.type = SUSTAIN;
      return cmd;
    }
    bool render(int nframes, Sample *output) {
      if (length_ == -1) {
	// sustained for the entire block
	std::fill_n(output,nframes,value_);
	return true;
      }
      for (int i=0; i<nframes; i++) {
      STEP:
	if (length_ > 0) {
	  output[i] = value_;
	  value_ += increment_;
	  --length_;
	}
	else {
	CMD:
	  if (ip_ == commands_.end()) {
	    std::fill(output+i,output+nframes,0);
	    return false;
	  }
	  else {
	    const EnvCommand &cmd = *ip_++;
	    switch (cmd.type) {
	    case SET:
	      value_ = cmd.set.value;
	      goto CMD;
	    case SLIDE:
	      length_ = sl::sampleRate()*cmd.slide.time;
	      increment_ = (cmd.slide.target - value_) / length_;
	      goto STEP;
	    case SUSTAIN:
	      length_ = -1;
	      increment_ = 0;
	      std::fill(output+i,output+nframes,value_);
	      return true;
	    default:
	      throw BadEnvCommandType();
	    }
	  }
	}
      }
      return true;
    }
    void add(EnvCommand cmd) {
      commands_.push_back(cmd);
    }
    void play() {
      ip_ = commands_.begin();
      length_ = 0;
      value_ = 0;
      increment_ = 0;
      length_ = 0;
    }
    void release(int delay) {
      if (length_ == -1) {
	// we are sustaining the note
	// advance to next command after `delay' samples
	length_ = delay;
      }
      else {
	// look for next sustain command
	EnvCommandVec::const_iterator pos;
	for (pos=ip_; pos != commands_.end(); ++pos) {
	  const EnvCommand &cmd = *pos;
	  if (cmd.type == SUSTAIN) break;
	}
	if (pos != commands_.end()) {
	  // found: jump to command following it
	  ip_ = pos+1;
	  length_ = delay;
	}
      }
    }
    EnvCommand& operator[](const int i) {
      return commands_[i];
    }
    Env& operator=(const Env &other) {
      if (this != &other) {
        commands_ = other.commands_;
      }
      return *this;
    }
  };

}

#endif
