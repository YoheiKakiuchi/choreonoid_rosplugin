#ifndef CNOID_ROS_PLUGIN_URDF_ITEM_H
#define CNOID_ROS_PLUGIN_URDF_ITEM_H

#include <cnoid/Item>

#include "exportdecl.h"

namespace cnoid {

class URDFSaveItemImpl;

class CNOID_EXPORT URDFSaveItem : public Item
{
public:
  static void initialize(ExtensionManager* ext);

  URDFSaveItem();
  URDFSaveItem(const URDFSaveItem& org);
  virtual ~URDFSaveItem();

  bool saveURDF(const std::string &filename);
protected:
  virtual Item* doDuplicate() const override;
//virtual void onPositionChanged() override;
//virtual void doPutProperties(PutPropertyFunction& putProperty) override;
//virtual bool store(Archive& archive) override;
//virtual bool restore(const Archive& archive) override;
private:
  URDFSaveItemImpl *impl;
};

typedef ref_ptr<URDFSaveItem> URDFSaveItemPtr;

}

#endif
