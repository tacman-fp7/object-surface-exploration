// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <robotControl.h>
#include <yarp/os/idl/WireTypes.h>



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

class robotControl_explore : public yarp::os::Portable {
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

bool robotControl_explore::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("explore",1,1)) return false;
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

void robotControl_explore::init() {
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
bool robotControl::explore() {
  bool _return = false;
  robotControl_explore helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::explore()");
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
    if (tag == "explore") {
      bool _return;
      _return = explore();
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
    helpString.push_back("approach");
    helpString.push_back("contact");
    helpString.push_back("explore");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="approach") {
      helpString.push_back("bool approach() ");
    }
    if (functionName=="contact") {
      helpString.push_back("bool contact() ");
    }
    if (functionName=="explore") {
      helpString.push_back("bool explore() ");
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


