/**
 * @file LibMultiSense/MultiSenseChannel.hh
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
 *   2013-04-25, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_MultiSenseChannel_hh
#define LibMultiSense_MultiSenseChannel_hh

#include <string>
#include <stdint.h>

#include "MultiSenseTypes.hh"

namespace crl {
namespace multisense {

class Channel {
public:

    //
    // Create an instance
    //
    // 'sensorAddress' can be a dotted-quad, or any hostname 
    // resolvable by gethostbyname().

    static Channel* Create(const std::string& sensorAddress);

    //
    // Destroy an instance

    static void Destroy(Channel *instanceP);

    //
    // Callback registration
    //
    // Each call will create a unique internal thread dedicated 
    // to the callback.
    //
    // Pointers to sensor data in the callback are no longer
    // valid after returning from the callback, unless reserved
    // by the user (see reserveCallbackBuffer() below.)
    //
    // Sensor data are queued per-callback, however, the queue
    // depth is limited, and the oldest data will be silently dropped 
    // if the callback falls behind.
    //
    // Adding multiple callbacks of the same data type is allowed (the 
    // same instance of sensor data will be presented to each callback.)
    //
    // For images:
    //
    //    Multiple image types may be subscribed to simultaneously in
    //    a single callback using the 'imageTypeMask'. 
    //
    //    Mutliple callbacks of differing types may be added in order 
    //    to isolate image processing by thread.
    //
    // For PPS events:
    //
    //    The queue depth is limited to 1 event. 

    virtual Status addIsolatedCallback(image::Callback callback, 
                                       DataSource      imageSourceMask,
                                       void           *userDataP=NULL) = 0;

    virtual Status addIsolatedCallback(lidar::Callback callback,
                                       void           *userDataP=NULL) = 0;

    virtual Status addIsolatedCallback(pps::Callback callback,
                                       void         *userDataP=NULL) = 0;

    //
    // Callback deregistration

    virtual Status removeIsolatedCallback(image::Callback callback) = 0;
    virtual Status removeIsolatedCallback(lidar::Callback callback) = 0;
    virtual Status removeIsolatedCallback(pps::Callback callback) = 0;

    //
    // Callback buffer reservation.
    //
    // The memory buffer behind a sensor datum within an isolated callback
    // may be reserved by the user.  This is useful for performing data
    // processing outside of the channel callback, without having to perform
    // a memory copy of the sensor data.
    //
    // The channel is guaranteed not to reuse the memory buffer until the
    // user releases it back.  Note that there are a limited amount of memory
    // buffers, and care should be taken not to reserve them for too long.
    //
    // reserveCallbackBuffer() will return a valid (non NULL) reference only 
    // when called within the context of a channel callback.
    //
    // releaseCallbackBuffer() may be called from any thread context.

    virtual void  *reserveCallbackBuffer()                 = 0;
    virtual Status releaseCallbackBuffer(void *referenceP) = 0; 

    //
    // Histograms may be queried by frame ID from recently received images.
    //
    // The histogram data is stored on the heap and must be released by
    // calling releaseHistogram(). If the histogram for the ID is not
    // found in the channel's internal cache, NULL will be returned.

    virtual const uint32_t *getHistogram(int64_t    frameId,
                                         uint32_t&  channels,
                                         uint32_t&  bins) = 0;
    virtual Status      releaseHistogram(int64_t frameId) = 0;

    //
    // Enable or disable local network-based time synchronization.
    //
    // Each Channel will keep a continuously updating and filtered offset between 
    // the sensor's internal clock and the local system clock.
    //
    // If enabled, all sensor timestamps will be reported in the local system
    // clock frame, after the offset has been applied.
    //
    // If disabled, all sensor timestamps will be reported in the frame of the
    // sensor's clock, which is free-running from 0 seconds on power up.
    //
    // The network-based time synchronization is enabled by default.

    virtual Status networkTimeSynchronization(bool enabled) = 0;

    //
    // Control/query

    virtual Status startStreams        (DataSource mask)                    = 0;
    virtual Status stopStreams         (DataSource mask)                    = 0;
    virtual Status getEnabledStreams   (DataSource& mask)                   = 0;

    virtual Status setTriggerSource    (TriggerSource s)                    = 0;

    virtual Status setMotorSpeed       (float rpm)                          = 0;

    virtual Status getLightingConfig   (lighting::Config& c)                = 0;
    virtual Status setLightingConfig   (const lighting::Config& c)          = 0;

    virtual Status getSensorVersion    (VersionType& version)               = 0;
    virtual Status getApiVersion       (VersionType& version)               = 0;
    virtual Status getVersionInfo      (system::VersionInfo& v)             = 0;

    virtual Status getImageConfig      (image::Config& c)                   = 0;
    virtual Status setImageConfig      (const image::Config& c)             = 0;

    virtual Status getImageCalibration (image::Calibration& c)              = 0;
    virtual Status setImageCalibration (const image::Calibration& c)        = 0;

    virtual Status getLidarCalibration (lidar::Calibration& c)              = 0;
    virtual Status setLidarCalibration (const lidar::Calibration& c)        = 0;

    //
    // System configuration

    virtual Status getDeviceModes      (std::vector<system::DeviceMode>& m) = 0;

    virtual Status getMtu              (int32_t& mtu)                       = 0;
    virtual Status setMtu              (int32_t mtu)                        = 0;

    virtual Status getNetworkConfig    (system::NetworkConfig& c)           = 0;
    virtual Status setNetworkConfig    (const system::NetworkConfig& c)     = 0;

    virtual Status getDeviceInfo       (system::DeviceInfo& info)           = 0;
    virtual Status setDeviceInfo       (const std::string& key,
                                        const system::DeviceInfo& i)        = 0;

    virtual Status flashBitstream      (const std::string& file)            = 0;
    virtual Status flashFirmware       (const std::string& file)            = 0;

    virtual Status verifyBitstream     (const std::string& file)            = 0;
    virtual Status verifyFirmware      (const std::string& file)            = 0;
};


}; // namespace multisense
}; // namespace crl

#endif // LibMultiSense_MultiSenseChannel_hh
