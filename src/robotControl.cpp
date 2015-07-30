// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <robotControl.h>
#include <yarp/os/idl/WireTypes.h>



class robotControl_setHomePose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_goToHomePose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_setStartingPose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_setEndPose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_goToStartingPose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_goToEndPose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_explore : public yarp::os::Portable {
public:
  bool onOff;
  bool _return;
  void init(const bool onOff);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_updateHomePose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_updateContactPose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_approach : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_contact : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool robotControl_setHomePose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("setHomePose",1,1)) return false;
  return true;
}

bool robotControl_setHomePose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_setHomePose::init() {
  _return = false;
}

bool robotControl_goToHomePose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("goToHomePose",1,1)) return false;
  return true;
}

bool robotControl_goToHomePose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_goToHomePose::init() {
  _return = false;
}

bool robotControl_setStartingPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("setStartingPose",1,1)) return false;
  return true;
}

bool robotControl_setStartingPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_setStartingPose::init() {
  _return = false;
}

bool robotControl_setEndPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("setEndPose",1,1)) return false;
  return true;
}

bool robotControl_setEndPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_setEndPose::init() {
  _return = false;
}

bool robotControl_goToStartingPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("goToStartingPose",1,1)) return false;
  return true;
}

bool robotControl_goToStartingPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_goToStartingPose::init() {
  _return = false;
}

bool robotControl_goToEndPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("goToEndPose",1,1)) return false;
  return true;
}

bool robotControl_goToEndPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_goToEndPose::init() {
  _return = false;
}

bool robotControl_explore::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("explore",1,1)) return false;
  if (!writer.writeBool(onOff)) return false;
  return true;
}

bool robotControl_explore::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_explore::init(const bool onOff) {
  _return = false;
  this->onOff = onOff;
}

bool robotControl_updateHomePose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("updateHomePose",1,1)) return false;
  return true;
}

bool robotControl_updateHomePose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_updateHomePose::init() {
  _return = false;
}

bool robotControl_updateContactPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("updateContactPose",1,1)) return false;
  return true;
}

bool robotControl_updateContactPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_updateContactPose::init() {
  _return = false;
}

bool robotControl_approach::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("approach",1,1)) return false;
  return true;
}

bool robotControl_approach::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_approach::init() {
  _return = false;
}

bool robotControl_contact::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("contact",1,1)) return false;
  return true;
}

bool robotControl_contact::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_contact::init() {
  _return = false;
}

bool robotControl_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool robotControl_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_quit::init() {
  _return = false;
}

robotControl::robotControl() {
  yarp().setOwner(*this);
}
bool robotControl::setHomePose() {
  bool _return = false;
  robotControl_setHomePose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::setHomePose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::goToHomePose() {
  bool _return = false;
  robotControl_goToHomePose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::goToHomePose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::setStartingPose() {
  bool _return = false;
  robotControl_setStartingPose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::setStartingPose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::setEndPose() {
  bool _return = false;
  robotControl_setEndPose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::setEndPose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::goToStartingPose() {
  bool _return = false;
  robotControl_goToStartingPose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::goToStartingPose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::goToEndPose() {
  bool _return = false;
  robotControl_goToEndPose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::goToEndPose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::explore(const bool onOff) {
  bool _return = false;
  robotControl_explore helper;
  helper.init(onOff);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::explore(const bool onOff)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::updateHomePose() {
  bool _return = false;
  robotControl_updateHomePose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::updateHomePose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::updateContactPose() {
  bool _return = false;
  robotControl_updateContactPose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::updateContactPose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::approach() {
  bool _return = false;
  robotControl_approach helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::approach()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::contact() {
  bool _return = false;
  robotControl_contact helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::contact()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::quit() {
  bool _return = false;
  robotControl_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool robotControl::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "setHomePose") {
      bool _return;
      _return = setHomePose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "goToHomePose") {
      bool _return;
      _return = goToHomePose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setStartingPose") {
      bool _return;
      _return = setStartingPose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setEndPose") {
      bool _return;
      _return = setEndPose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "goToStartingPose") {
      bool _return;
      _return = goToStartingPose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "goToEndPose") {
      bool _return;
      _return = goToEndPose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "explore") {
      bool onOff;
      if (!reader.readBool(onOff)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = explore(onOff);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "updateHomePose") {
      bool _return;
      _return = updateHomePose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "updateContactPose") {
      bool _return;
      _return = updateContactPose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "approach") {
      bool _return;
      _return = approach();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "contact") {
      bool _return;
      _return = contact();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> robotControl::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("setHomePose");
    helpString.push_back("goToHomePose");
    helpString.push_back("setStartingPose");
    helpString.push_back("setEndPose");
    helpString.push_back("goToStartingPose");
    helpString.push_back("goToEndPose");
    helpString.push_back("explore");
    helpString.push_back("updateHomePose");
    helpString.push_back("updateContactPose");
    helpString.push_back("approach");
    helpString.push_back("contact");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setHomePose") {
      helpString.push_back("bool setHomePose() ");
    }
    if (functionName=="goToHomePose") {
      helpString.push_back("bool goToHomePose() ");
    }
    if (functionName=="setStartingPose") {
      helpString.push_back("bool setStartingPose() ");
    }
    if (functionName=="setEndPose") {
      helpString.push_back("bool setEndPose() ");
    }
    if (functionName=="goToStartingPose") {
      helpString.push_back("bool goToStartingPose() ");
    }
    if (functionName=="goToEndPose") {
      helpString.push_back("bool goToEndPose() ");
    }
    if (functionName=="explore") {
      helpString.push_back("bool explore(const bool onOff) ");
    }
    if (functionName=="updateHomePose") {
      helpString.push_back("bool updateHomePose() ");
    }
    if (functionName=="updateContactPose") {
      helpString.push_back("bool updateContactPose() ");
    }
    if (functionName=="approach") {
      helpString.push_back("bool approach() ");
    }
    if (functionName=="contact") {
      helpString.push_back("bool contact() ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


