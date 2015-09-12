// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <set>
#include <vector>

#include <boost/noncopyable.hpp>

#include "quaternion.h"
#include "point3d.h"

///@file
///
/// All coordinates assume the following conventions.
///  +x - right
///  +y - forward
///  +z - up

namespace legtool {

struct Frame;

/// A Transform represents a mapping from one 6dof reference frame to
/// another.  It is constructed from a translation and rotation.  When
/// applying the transform to points, the rotation is applied first,
/// then the translation.
struct Transform {
  Point3D translation;
  Quaternion rotation;

  Point3D Apply(const Point3D& other) const {
    return rotation.Rotate(other) + translation;
  }

  Transform Apply(const Transform& other) const {
    return Transform{
        translation + rotation.Rotate(other.translation),
        rotation * other.rotation};
  }

  template <typename T>
  T Inverse(const T& other) const {
    return Inversed().Apply(other);
  }

  Transform Inversed() const {
    auto conjugated = rotation.conjugated();
    return Transform{
        conjugated.Rotate(translation.scaled(-1.0)),
        conjugated};
  }

  Transform() {}
  Transform(const Point3D& translation, const Quaternion& rotation)
      : translation(translation), rotation(rotation) {}

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(translation));
    a->Visit(LT_NVP(rotation));
  }
};


/// This defines the relationship between coordinates specified in one
/// frame, and a parent frame.
///
/// Defined as non-copyable, because FrameRelationships rely on
/// object references, and copying them around would foil that.
struct Frame : boost::noncopyable {
  Transform transform;
  const Frame* parent;

  Frame() : transform(), parent(nullptr) {}

  Frame(Point3D translation,
        Quaternion rotation,
        const Frame* parent=nullptr)
      : transform{translation, rotation},
        parent(parent) {}

  std::string str() const {
    return (boost::format("<Frame t=%s r=%s>") %
            transform.translation.str() %
            transform.rotation.str()).str();
  }

  template <typename T>
  T MapToParent(const T& t) const { return transform.Apply(t); }

  template <typename T>
  T MapFromParent(const T& t) const { return transform.Inverse(t); }

  Transform TransformToFrame(const Frame* other) const {
    return Relation(this, other).transform();
  }

  template <typename T>
  T MapToFrame(const Frame* frame, const T& other) const {
    return TransformToFrame(frame).Apply(other);
  }

  template <typename T>
  T MapFromFrame(const Frame* frame, const T& other) const {
    return TransformToFrame(frame).Inversed().Apply(other);
  }

  /// This maintains the relationship between two frames, and allows a
  /// transform between the two frames to be re-calculated over time
  /// as it may change.
  class Relation : boost::noncopyable {
   public:
    Relation(const Frame* source, const Frame* destination) {
      // TODO jpieper: It would be nice if this calculation didn't
      // require so much dynamic memory allocation.
      std::set<const Frame*> source_parents_dict;
      for (const Frame* node = source; node; node = node->parent) {
        source_parents_list_.push_back(node);
        source_parents_dict.insert(node);
      }

      for (const Frame* node = destination; node; node = node->parent) {
        if (source_parents_dict.count(node)) {
          auto it = std::find(source_parents_list_.begin(),
                              source_parents_list_.end(),
                              node);
          BOOST_ASSERT(it != source_parents_list_.end());

          source_parents_list_.erase(it, source_parents_list_.end());
          break;
        }
        destination_parents_list_.push_back(node);
      }
    }

    Transform transform() const {
      Transform result;
      for (const Frame* node: source_parents_list_) {
        result = node->MapToParent(result);
      }
      for (auto it = destination_parents_list_.rbegin();
           it != destination_parents_list_.rend();
           ++it) {
        result = (*it)->MapFromParent(result);
      }
      return result;
    }

   private:
    std::vector<const Frame*> source_parents_list_;
    std::vector<const Frame*> destination_parents_list_;
  };
};
}
