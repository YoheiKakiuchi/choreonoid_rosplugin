#define _USE_MATH_DEFINES
#include "URDFSaveItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/ItemManager>

#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>

#include <iostream>
#include <cmath>
#include "gettext.h"

//
#include <urdf_parser/urdf_parser.h>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

#define USE_LINK_PREFIX 1

namespace {

// mesh extractor
struct Triangle {
  int indices[3];
};

bool saveBodyItem(URDFSaveItem* item, const std::string& filename)
{
  std::cerr << "save: " << filename << std::endl;
  return item->saveURDF(filename);
}

}

namespace cnoid {
class URDFSaveItemImpl {

public:
  URDFSaveItemImpl() : body(nullptr) {
  }
  URDFSaveItemImpl(Body *b) : body(b) {
  }
  void setBody(Body *b) { body = b; }
  bool saveURDF(const std::string &filename);

  void addMesh(MeshExtractor* extractor,
               std::vector<Affine3> &poses,
               std::vector<urdf::GeometrySharedPtr> &geoms);

private:
  Body *body;
};
}

void URDFSaveItem::initialize(ExtensionManager* ext)
{
  ext->itemManager().registerClass<URDFSaveItem>("URDFSaveItem");
  ext->itemManager().addCreationPanel<URDFSaveItem>();
  ext->itemManager().addSaver<URDFSaveItem>(
         _("urdf"), "URDF-MODEL", "urdf",
         std::bind(saveBodyItem, _1, _2));
}

URDFSaveItem::URDFSaveItem()
{
  impl = new URDFSaveItemImpl();
}

URDFSaveItem::URDFSaveItem(const URDFSaveItem& org)
    : Item(org)
{
  impl = org.impl;
}

URDFSaveItem::~URDFSaveItem()
{
  delete impl;
}

bool URDFSaveItem::saveURDF(const std::string &filename)
{
  Item *p = parentItem();
  if(!!p) {
    //std::cerr << "parent: " << p->name() << std::endl;
    BodyItem* casted = dynamic_cast<BodyItem*>(p);
    if (!!casted) {
      //std::cerr << "BodyItem" << std::endl;
      impl->setBody(casted->body());
    }
  }
  return impl->saveURDF(filename);
}

Item* URDFSaveItem::doDuplicate() const
{
  return new URDFSaveItem(*this);
}

bool URDFSaveItemImpl::saveURDF(const std::string &filename)
{
  if(!body) return false;

  std::cerr << "parse" << std::endl;

  urdf::ModelInterface _urdf_model;
  _urdf_model.clear();

  _urdf_model.name_ = body->name();

  // store q-vector
  std::vector<double> qvec(body->numJoints());
  for (int idx = 0; idx < body->numJoints(); idx++) {
    Link *jt = body->joint(idx);
    qvec[idx] = jt->q();
    jt->q() = 0;
  }
  //body->updateLinkTree();
  body->calcForwardKinematics();

  // dump links
  for (int idx = 0; idx < body->numLinks(); idx++) {
    urdf::LinkSharedPtr ulk(new urdf::Link());
    ulk->clear();

    Link *lk = body->link(idx);
#if USE_LINK_PREFIX
    ulk->name = "LK_" + lk->name();
#else
    ulk->name = lk->name();
#endif
    ulk->inertial.reset(new urdf::Inertial());
    ulk->inertial->mass = lk->mass();
    const Vector3 &com = lk->centerOfMass();
    ulk->inertial->origin.position.x = com.x();
    ulk->inertial->origin.position.y = com.y();
    ulk->inertial->origin.position.z = com.z();
    const Matrix3 &Iner = lk->I(); // inertia tensor (self local, around c)
    ulk->inertial->ixx = Iner(0,0);
    ulk->inertial->ixy = Iner(0,1);
    ulk->inertial->ixz = Iner(0,2);
    ulk->inertial->iyy = Iner(1,1);
    ulk->inertial->iyz = Iner(1,2);
    ulk->inertial->izz = Iner(2,2);
    // shape
#if 1
    if (lk->visualShape()) {
      //std::cerr << "visual!!" << std::endl;
      //
      MeshExtractor* extractor = new MeshExtractor;
      std::vector<Affine3> poses;
      std::vector<urdf::GeometrySharedPtr> geoms;
      if(extractor->extract(lk->collisionShape(),
                            [&]() { addMesh(extractor, poses, geoms); } )) {
        //
      }
      if (geoms.size() > 0 && poses.size() == geoms.size()) {
        for(int i = 0; i < geoms.size(); i++) {
          urdf::VisualSharedPtr vis(new urdf::Visual());
          Translation3 tt(poses[i].translation());
          vis->origin.position.x = tt.x();
          vis->origin.position.y = tt.y();
          vis->origin.position.z = tt.z();
          Quaternion qq(poses[i].linear());
          vis->origin.rotation.setFromQuaternion(qq.x(), qq.y(), qq.z(), qq.w());
          vis->geometry = geoms[i];
          ulk->visual_array.push_back(vis);
        }
        ulk->visual = ulk->visual_array[0];
      }
      delete extractor;
    }
#endif
    if (lk->collisionShape()) {
      //std::cerr << "collision!!" << std::endl;
      MeshExtractor* extractor = new MeshExtractor;
      std::vector<Affine3> poses;
      std::vector<urdf::GeometrySharedPtr> geoms;
      if(extractor->extract(lk->collisionShape(),
                            [&]() { addMesh(extractor, poses, geoms); } )) {
        //
      }
      if (geoms.size() > 0 && poses.size() == geoms.size()) {
        for(int i = 0; i < geoms.size(); i++) {
          urdf::CollisionSharedPtr col(new urdf::Collision());
          Translation3 tt(poses[i].translation());
          col->origin.position.x = tt.x();
          col->origin.position.y = tt.y();
          col->origin.position.z = tt.z();
          Quaternion qq(poses[i].linear());
          col->origin.rotation.setFromQuaternion(qq.x(), qq.y(), qq.z(), qq.w());
          col->geometry = geoms[i];
          ulk->collision_array.push_back(col);
        }
        ulk->collision = ulk->collision_array[0];
      }
      delete extractor;
    }
    _urdf_model.links_.insert(std::make_pair(ulk->name, ulk));
  }

  // dump joints
  for (int idx = 0; idx < body->numLinks(); idx++) {
    Link *lk = body->link(idx);
    if (lk->isRoot()) {
#if USE_LINK_PREFIX
      _urdf_model.getLink("LK_" + lk->name(), _urdf_model.root_link_);
#else
      _urdf_model.getLink(lk->name(), _urdf_model.root_link_);
#endif
      continue;
    }
    Link *plk = lk->parent();
    if (!plk) {
      std::cerr << "NO Parent, but this is not ROOT, " << lk->name() << std::endl;
      continue;
    }
    urdf::JointSharedPtr jt(new urdf::Joint());
    jt->name = lk->name();
#if USE_LINK_PREFIX
    jt->child_link_name  = "LK_" + lk->name();
    jt->parent_link_name = "LK_" + plk->name();
#else
    jt->child_link_name  = lk->name();
    jt->parent_link_name = plk->name();
#endif
    { // add link connections
      urdf::LinkSharedPtr u_clk;
      urdf::LinkSharedPtr u_plk;
#if USE_LINK_PREFIX
      _urdf_model.getLink("LK_" + lk->name(), u_clk);
      _urdf_model.getLink("LK_" + plk->name(), u_plk);
#else
      _urdf_model.getLink(lk->name(), u_clk);
      _urdf_model.getLink(plk->name(), u_plk);
#endif
      u_clk->parent_joint = jt;
      u_clk->setParent(u_plk);

      u_plk->child_joints.push_back(jt);
      u_plk->child_links.push_back(u_clk);
    }

    switch(lk->jointType()) {
    case Link::REVOLUTE_JOINT:
      //case Link::ROTATIONAL_JOINT:
      jt->type = urdf::Joint::REVOLUTE;
      break;
    case Link::PRISMATIC_JOINT:
      //case Link::SLIDE_JOINT:
      jt->type = urdf::Joint::PRISMATIC;
      break;
    case Link::FIXED_JOINT:
      jt->type = urdf::Joint::FIXED;
      break;
    case Link::FREE_JOINT:
      jt->type = urdf::Joint::FLOATING;
      break;
    }
    jt->axis.x = lk->jointAxis().x();
    jt->axis.y = lk->jointAxis().y();
    jt->axis.z = lk->jointAxis().z();

    Position pT = plk->position();
    Position cT = lk->position();
    Position rel_p_c = pT.inverse() * cT;

    Position::TranslationPart rel_t = rel_p_c.translation();
    jt->parent_to_joint_origin_transform.position.x = rel_t.x();
    jt->parent_to_joint_origin_transform.position.y = rel_t.y();
    jt->parent_to_joint_origin_transform.position.z = rel_t.z();
    Quaternion rel_q(rel_p_c.linear());
    jt->parent_to_joint_origin_transform.rotation.setFromQuaternion(
                         rel_q.x(), rel_q.y(), rel_q.z(), rel_q.w());
    //lk->q_initial();
    jt->limits.reset(new urdf::JointLimits());
    jt->limits->upper = lk->q_upper();
    jt->limits->lower = lk->q_lower();
    {
      double dq_u = std::abs(lk->dq_upper());
      double dq_l = std::abs(lk->dq_lower());
      if (dq_u > dq_l) {
        jt->limits->velocity = dq_u;
      } else {
        jt->limits->velocity = dq_l;
      }
      if (jt->limits->velocity == 0.0) {
        std::cerr << "[WARN] joint: " << jt->name << " does not have velocity limits!" << std::endl;
        jt->limits->velocity = 1.0;
      }
    }
    // TODO:
    jt->limits->effort = 100;

    _urdf_model.joints_.insert(std::make_pair(jt->name, jt));
  }
  // dump_devices

  // restore q-vector
  for (int idx=0; idx < body->numJoints(); idx++) {
    body->joint(idx)->q() = qvec[idx];
  }
  //body->updateLinkTree();
  body->calcForwardKinematics();

  TiXmlDocument *doc = urdf::exportURDF(_urdf_model);

  return doc->SaveFile(filename);
}

void URDFSaveItemImpl::addMesh(MeshExtractor* extractor,
                               std::vector<Affine3> &poses,
                               std::vector<urdf::GeometrySharedPtr> &geoms)
{
  SgMesh* mesh = extractor->currentMesh();
  const Affine3& T = extractor->currentTransform();
#if 0
  {
    std::cerr << "  currentT: ";
    Vector3 v(T.translation());
    Quaternion q = Quaternion(T.linear());
    std::cerr << v.x() << ", " << v.y() << ", " << v.z() << " / "
              << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
  }
#endif
  bool meshAdded = false;

  if(mesh->primitiveType() != SgMesh::MESH) {
    //std::cerr << "  not mesh!" << std::endl;
    bool doAddPrimitive = false;
    Vector3 scale;
    boost::optional<Vector3> translation;
    // scale
    if(!extractor->isCurrentScaled()){
      scale.setOnes();
      doAddPrimitive = true;
    } else {
      std::cerr << "    scaled!" << std::endl;
      Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
        extractor->currentTransform();

      if(S.linear().isDiagonal()) {
        if(!S.translation().isZero()) {
          translation = S.translation();
        }
        scale = S.linear().diagonal();
        if(mesh->primitiveType() == SgMesh::BOX) {
          doAddPrimitive = true;
        } else if(mesh->primitiveType() == SgMesh::SPHERE) {
          // check if the sphere is uniformly scaled for all the axes
          if(scale.x() == scale.y() && scale.x() == scale.z()) {
            doAddPrimitive = true;
          }
        } else if(mesh->primitiveType() == SgMesh::CYLINDER ||
                  mesh->primitiveType() == SgMesh::CAPSULE ) {
          // check if the bottom circle face is uniformly scaled
          if(scale.x() == scale.z()) {
            doAddPrimitive = true;
          }
        }
      }
      //std::cerr << "    scaled!" << std::endl;
    }

    if (doAddPrimitive) {
      //bool created = false;
      urdf::GeometrySharedPtr geom;

      switch(mesh->primitiveType()){
      case SgMesh::BOX : {
        const Vector3& s = mesh->primitive<SgMesh::Box>().size;
        //std::cerr << "   -> box" << std::endl;
        urdf::Box *b = new urdf::Box();
        b->dim.x = s.x() * scale.x();
        b->dim.y = s.y() * scale.y();
        b->dim.z = s.z() * scale.z();
        geom.reset((urdf::Geometry *)b);
        break; }
      case SgMesh::SPHERE : {
        SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
        //sphere.radius * scale.x()
        //std::cerr << "   -> sphere" << std::endl;
        urdf::Sphere *b = new urdf::Sphere();
        b->radius = sphere.radius * scale.x();
        geom.reset((urdf::Geometry *)b);
        break; }
      case SgMesh::CYLINDER : {
        SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
        //cylinder.radius * scale.x(), cylinder.height * scale.y()
        //std::cerr << "   -> cylinder" << std::endl;
        urdf::Cylinder *b = new urdf::Cylinder();
        b->radius = cylinder.radius * scale.x();
        b->length = cylinder.height * scale.y();
        geom.reset((urdf::Geometry *)b);
        break; }
#if 0
      case SgMesh::CAPSULE : {
        SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
        //capsule.radius * scale.x(), capsule.height * scale.y()
        std::cerr << "   -> capsule" << std::endl;
        created = true;
        break; }
#endif
      default :
        break;
      }
      if (!!geom) {
        // urdf:cylinder direction: z, origin: center of z
        // body:cylinder direction: y, origin: center of y
        Affine3 T_ = extractor->currentTransformWithoutScaling();
        if(mesh->primitiveType()==SgMesh::CYLINDER ||
           mesh->primitiveType()==SgMesh::CAPSULE )
          T_ *= AngleAxis(M_PI_2, Vector3::UnitX());

        if(translation) {
          T_ *= Translation3(*translation);
        }
        geoms.push_back(geom);
        poses.push_back(T_);
        meshAdded = true;
      } // if (created)
    }
  } // if(mesh->primitiveType() != SgMesh::MESH) {
  if( !meshAdded ) {
    std::cerr << "  mesh!" << std::endl;
    std::vector<Vector3>  vertices;
    std::vector<Triangle> triangles;

    const SgVertexArray& vertices_ = *mesh->vertices();
    const int numVertices = vertices_.size();

    for(int i = 0; i < numVertices; ++i) {
      //const Vector3 v = T * vertices_[i].cast<Position::Scalar>() - link->c();
      const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
      vertices.push_back(Vector3(v.x(), v.y(), v.z()));
      std::cerr << "   vtc[" << i << "]: "
                << v.x() << ", "
                << v.y() << ", "
                << v.z() << std::endl;
    }

    const int numTriangles = mesh->numTriangles();
    for(int i = 0; i < numTriangles; ++i) {
      SgMesh::TriangleRef src = mesh->triangle(i);
      Triangle tri;
      tri.indices[0] = src[0];
      tri.indices[1] = src[1];
      tri.indices[2] = src[2];
      triangles.push_back(tri);
      std::cerr << "   idx: " <<  tri.indices[0] << " "
                << tri.indices[1] << " "
                << tri.indices[2] << std::endl;
    }

    // store mesh...
  }
}
