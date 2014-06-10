/**
 * @file LibMultiSense/JpegMessage.h
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2013-06-12, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_JpegMessage
#define LibMultiSense_JpegMessage

#include <typeinfo>
#include <cmath>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ JpegImageHeader {
public:

static const IdType      ID      = ID_DATA_JPEG_IMAGE;
static const VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    uint32_t source;
    int64_t  frameId;
    uint16_t width;
    uint16_t height;
    uint32_t length;
    uint32_t quality;

    JpegImageHeader() : 
#ifdef SENSORDPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        source(0),
        frameId(0),
        width(0),
        height(0),
        length(0),
        quality(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class JpegImage : public JpegImageHeader {
public:

    void *dataP;

    //
    // Constructors

    JpegImage(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    JpegImage() : dataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & source;
        message & frameId;
        message & width;
        message & height;
        message & length;
        message & quality;

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {
          
            message.write(dataP, length);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + length);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
