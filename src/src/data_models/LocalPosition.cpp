/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <data_models/LocalPosition.h>
#include <fmt/format.h>

namespace AtlasFusion::DataModels {


    rtl::RigidTf3D<double> LocalPosition::ToTf() const {
        return {GetOrientation(), GetPosition()};
    }

    LocalPosition LocalPosition::operator+(LocalPosition &other) {

        return LocalPosition{{position_.x() + other.GetPosition().x(),
                              position_.y() + other.GetPosition().y(),
                              position_.z() + other.GetPosition().z()},
                             {orientation_ * other.GetOrientation()},
                             timestamp_ + other.GetTimestamp()};
    }

    LocalPosition LocalPosition::operator-(LocalPosition &other) {
        return LocalPosition{{position_.x() - other.GetPosition().x(),
                              position_.y() - other.GetPosition().y(),
                              position_.z() - other.GetPosition().z()},
                             {other.GetOrientation().inverted() * orientation_},
                             timestamp_ - other.GetTimestamp()};
    }

    std::string LocalPosition::ToString() {
        return fmt::format("{} {} {} | {} {} {} {} | {}",
                           position_.x(), position_.y(), position_.z(), orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w(), timestamp_);
    }

}