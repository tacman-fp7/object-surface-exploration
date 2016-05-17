// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <robotControl.h>
#include <yarp/os/idl/WireTypes.h>



class robotControl_setStartingPose : public yarp::os::Portable {
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

class robotControl_setEndPose : public yarp::os::Portable {
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

class robotControl_startExploring : public yarp::os::Portable {
public:
  std::string type;
  std::string objectName;
  bool _return;
  void init(const std::string& type, const std::string& objectName);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_stopExploring : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_fingerSetAngle : public yarp::os::Portable {
public:
  double angle;
  bool _return;
  void init(const double angle);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_prepHand : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_openHand : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_calibrateHand : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_enableSurfaceSampling : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_disableSurfaceSampling : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_refineModelEnable : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_refineModelDisable : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_nRepeatsSet : public yarp::os::Portable {
public:
  int32_t nRepeats;
  bool _return;
  void init(const int32_t nRepeats);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_validatePositionsEnable : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_validatePositionsDisable : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class robotControl_setHeight : public yarp::os::Portable {
public:
  double height;
  bool _return;
  void init(const double height);
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

bool robotControl_startExploring::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("startExploring",1,1)) return false;
  if (!writer.writeString(type)) return false;
  if (!writer.writeString(objectName)) return false;
  return true;
}

bool robotControl_startExploring::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_startExploring::init(const std::string& type, const std::string& objectName) {
  _return = false;
  this->type = type;
  this->objectName = objectName;
}

bool robotControl_stopExploring::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("stopExploring",1,1)) return false;
  return true;
}

bool robotControl_stopExploring::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_stopExploring::init() {
  _return = false;
}

bool robotControl_fingerSetAngle::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("fingerSetAngle",1,1)) return false;
  if (!writer.writeDouble(angle)) return false;
  return true;
}

bool robotControl_fingerSetAngle::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_fingerSetAngle::init(const double angle) {
  _return = false;
  this->angle = angle;
}

bool robotControl_prepHand::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("prepHand",1,1)) return false;
  return true;
}

bool robotControl_prepHand::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_prepHand::init() {
  _return = false;
}

bool robotControl_openHand::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("openHand",1,1)) return false;
  return true;
}

bool robotControl_openHand::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_openHand::init() {
  _return = false;
}

bool robotControl_calibrateHand::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("calibrateHand",1,1)) return false;
  return true;
}

bool robotControl_calibrateHand::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_calibrateHand::init() {
  _return = false;
}

bool robotControl_enableSurfaceSampling::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("enableSurfaceSampling",1,1)) return false;
  return true;
}

bool robotControl_enableSurfaceSampling::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_enableSurfaceSampling::init() {
  _return = false;
}

bool robotControl_disableSurfaceSampling::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("disableSurfaceSampling",1,1)) return false;
  return true;
}

bool robotControl_disableSurfaceSampling::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_disableSurfaceSampling::init() {
  _return = false;
}

bool robotControl_refineModelEnable::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("refineModelEnable",1,1)) return false;
  return true;
}

bool robotControl_refineModelEnable::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_refineModelEnable::init() {
  _return = false;
}

bool robotControl_refineModelDisable::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("refineModelDisable",1,1)) return false;
  return true;
}

bool robotControl_refineModelDisable::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_refineModelDisable::init() {
  _return = false;
}

bool robotControl_nRepeatsSet::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("nRepeatsSet",1,1)) return false;
  if (!writer.writeI32(nRepeats)) return false;
  return true;
}

bool robotControl_nRepeatsSet::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_nRepeatsSet::init(const int32_t nRepeats) {
  _return = false;
  this->nRepeats = nRepeats;
}

bool robotControl_validatePositionsEnable::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("validatePositionsEnable",1,1)) return false;
  return true;
}

bool robotControl_validatePositionsEnable::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_validatePositionsEnable::init() {
  _return = false;
}

bool robotControl_validatePositionsDisable::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("validatePositionsDisable",1,1)) return false;
  return true;
}

bool robotControl_validatePositionsDisable::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_validatePositionsDisable::init() {
  _return = false;
}

bool robotControl_setHeight::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setHeight",1,1)) return false;
  if (!writer.writeDouble(height)) return false;
  return true;
}

bool robotControl_setHeight::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void robotControl_setHeight::init(const double height) {
  _return = false;
  this->height = height;
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
bool robotControl::startExploring(const std::string& type, const std::string& objectName) {
  bool _return = false;
  robotControl_startExploring helper;
  helper.init(type,objectName);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::startExploring(const std::string& type, const std::string& objectName)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::stopExploring() {
  bool _return = false;
  robotControl_stopExploring helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::stopExploring()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::fingerSetAngle(const double angle) {
  bool _return = false;
  robotControl_fingerSetAngle helper;
  helper.init(angle);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::fingerSetAngle(const double angle)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::prepHand() {
  bool _return = false;
  robotControl_prepHand helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::prepHand()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::openHand() {
  bool _return = false;
  robotControl_openHand helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::openHand()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::calibrateHand() {
  bool _return = false;
  robotControl_calibrateHand helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::calibrateHand()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::enableSurfaceSampling() {
  bool _return = false;
  robotControl_enableSurfaceSampling helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::enableSurfaceSampling()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::disableSurfaceSampling() {
  bool _return = false;
  robotControl_disableSurfaceSampling helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::disableSurfaceSampling()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::refineModelEnable() {
  bool _return = false;
  robotControl_refineModelEnable helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::refineModelEnable()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::refineModelDisable() {
  bool _return = false;
  robotControl_refineModelDisable helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::refineModelDisable()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::nRepeatsSet(const int32_t nRepeats) {
  bool _return = false;
  robotControl_nRepeatsSet helper;
  helper.init(nRepeats);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::nRepeatsSet(const int32_t nRepeats)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::validatePositionsEnable() {
  bool _return = false;
  robotControl_validatePositionsEnable helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::validatePositionsEnable()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::validatePositionsDisable() {
  bool _return = false;
  robotControl_validatePositionsDisable helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::validatePositionsDisable()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool robotControl::setHeight(const double height) {
  bool _return = false;
  robotControl_setHeight helper;
  helper.init(height);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool robotControl::setHeight(const double height)");
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
    if (tag == "startExploring") {
      std::string type;
      std::string objectName;
      if (!reader.readString(type)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(objectName)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = startExploring(type,objectName);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stopExploring") {
      bool _return;
      _return = stopExploring();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "fingerSetAngle") {
      double angle;
      if (!reader.readDouble(angle)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = fingerSetAngle(angle);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "prepHand") {
      bool _return;
      _return = prepHand();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "openHand") {
      bool _return;
      _return = openHand();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibrateHand") {
      bool _return;
      _return = calibrateHand();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "enableSurfaceSampling") {
      bool _return;
      _return = enableSurfaceSampling();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "disableSurfaceSampling") {
      bool _return;
      _return = disableSurfaceSampling();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "refineModelEnable") {
      bool _return;
      _return = refineModelEnable();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "refineModelDisable") {
      bool _return;
      _return = refineModelDisable();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "nRepeatsSet") {
      int32_t nRepeats;
      if (!reader.readI32(nRepeats)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = nRepeatsSet(nRepeats);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "validatePositionsEnable") {
      bool _return;
      _return = validatePositionsEnable();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "validatePositionsDisable") {
      bool _return;
      _return = validatePositionsDisable();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setHeight") {
      double height;
      if (!reader.readDouble(height)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setHeight(height);
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
    helpString.push_back("setStartingPose");
    helpString.push_back("goToStartingPose");
    helpString.push_back("setEndPose");
    helpString.push_back("goToEndPose");
    helpString.push_back("startExploring");
    helpString.push_back("stopExploring");
    helpString.push_back("fingerSetAngle");
    helpString.push_back("prepHand");
    helpString.push_back("openHand");
    helpString.push_back("calibrateHand");
    helpString.push_back("enableSurfaceSampling");
    helpString.push_back("disableSurfaceSampling");
    helpString.push_back("refineModelEnable");
    helpString.push_back("refineModelDisable");
    helpString.push_back("nRepeatsSet");
    helpString.push_back("validatePositionsEnable");
    helpString.push_back("validatePositionsDisable");
    helpString.push_back("setHeight");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setStartingPose") {
      helpString.push_back("bool setStartingPose() ");
    }
    if (functionName=="goToStartingPose") {
      helpString.push_back("bool goToStartingPose() ");
    }
    if (functionName=="setEndPose") {
      helpString.push_back("bool setEndPose() ");
    }
    if (functionName=="goToEndPose") {
      helpString.push_back("bool goToEndPose() ");
    }
    if (functionName=="startExploring") {
      helpString.push_back("bool startExploring(const std::string& type, const std::string& objectName) ");
    }
    if (functionName=="stopExploring") {
      helpString.push_back("bool stopExploring() ");
    }
    if (functionName=="fingerSetAngle") {
      helpString.push_back("bool fingerSetAngle(const double angle) ");
    }
    if (functionName=="prepHand") {
      helpString.push_back("bool prepHand() ");
    }
    if (functionName=="openHand") {
      helpString.push_back("bool openHand() ");
    }
    if (functionName=="calibrateHand") {
      helpString.push_back("bool calibrateHand() ");
    }
    if (functionName=="enableSurfaceSampling") {
      helpString.push_back("bool enableSurfaceSampling() ");
    }
    if (functionName=="disableSurfaceSampling") {
      helpString.push_back("bool disableSurfaceSampling() ");
    }
    if (functionName=="refineModelEnable") {
      helpString.push_back("bool refineModelEnable() ");
    }
    if (functionName=="refineModelDisable") {
      helpString.push_back("bool refineModelDisable() ");
    }
    if (functionName=="nRepeatsSet") {
      helpString.push_back("bool nRepeatsSet(const int32_t nRepeats) ");
    }
    if (functionName=="validatePositionsEnable") {
      helpString.push_back("bool validatePositionsEnable() ");
    }
    if (functionName=="validatePositionsDisable") {
      helpString.push_back("bool validatePositionsDisable() ");
    }
    if (functionName=="setHeight") {
      helpString.push_back("bool setHeight(const double height) ");
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


