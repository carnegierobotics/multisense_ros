/**
 * @file LibMultiSense/SysDirectedStreamsMessage.h
 *
 * Copyright 2014
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
 *   2014-06-10, ekratzer@carnegierobotics.com, IR1069, created file.
 **/

#ifndef LibMultiSense_SysDirectedStreamsMessage
#define LibMultiSense_SysDirectedStreamsMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class DirectedStream {
public:
    static const VersionType VERSION = 1;

    uint32_t    mask;
    std::string address;
    uint16_t    udpPort;
    uint32_t    fpsDecimation;

    DirectedStream() {};
    DirectedStream(uint32_t           m,
                   const std::string& addr,
                   uint16_t           p,
                   uint32_t           dec) :
        mask(m),
        address(addr),
        udpPort(p),
        fpsDecimation(dec) {};

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        VersionType thisVersion = VERSION;

        message & thisVersion;
        message & mask;
        message & address;
        message & udpPort;
        message & fpsDecimation;
    };
};

class SysDirectedStreams {
public:
    static const IdType      ID        = ID_DATA_SYS_DIRECTED_STREAMS;
    static const VersionType VERSION   = 1;

    static const uint32_t    CMD_NONE  = 0;
    static const uint32_t    CMD_START = 1;
    static const uint32_t    CMD_STOP  = 2;

    uint32_t                          command;
    std::vector<wire::DirectedStream> streams;

    //
    // Constructors

    SysDirectedStreams(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysDirectedStreams(uint32_t cmd) : command(cmd) {};
    SysDirectedStreams() : command(CMD_NONE) {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & command;
        uint32_t elements = streams.size();
        message & elements;
        streams.resize(elements);
        for(uint32_t i=0; i<elements; i++)
            streams[i].serialize(message, version);
    }
};

}}}}; // namespaces

#endif
