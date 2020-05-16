#!/usr/bin/env python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Align video to robot logs, assuming that the side of the robot was
slapped in a unique pattern during the video recording.'''

import argparse
import numpy as np
import pylab
import subprocess
import tempfile

import scipy
import scipy.signal
import scipy.interpolate
import scipy.io
import scipy.io.wavfile

import mjlib.telemetry.file_reader as file_reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-v', '--video', type=str)
    parser.add_argument('-l', '--log', type=str)
    parser.add_argument('-s', '--video-start', type=float, default=0.0)
    parser.add_argument('-e', '--video-end', type=float, default=-1.0)
    parser.add_argument('--log-start', type=float, default=0.0)
    parser.add_argument('--log-end', type=float, default=-1.0)
    parser.add_argument('--distance', type=float, default=1.5,
                        help='distance in meters between robot and camera')
    parser.add_argument('--fudge', type=float, default=0.025)

    args = parser.parse_args()

    wav_file = tempfile.NamedTemporaryFile(suffix='.wav')

    subprocess.check_call(
        "ffmpeg -y -i {} -ac 1 -acodec pcm_s16le {} >/dev/null 2>&1".format(
            args.video, wav_file.name),
        shell = True)

    wav_data = scipy.io.wavfile.read(wav_file.name)
    audio_rate = wav_data[0]
    audio_data = wav_data[1] / 32767.0

    if args.video_end < 0.0:
        args.video_end = len(audio_data) / audio_rate


    fr = file_reader.FileReader(args.log)
    imu_data = [(x.data.timestamp, x.data.accel_mps2[1])
                for x in fr.items(['imu'])]
    imu_start_s = imu_data[0][0]
    imu_size_s = imu_data[-1][0] - imu_start_s

    if args.log_start < 0:
        args.log_start = imu_size_s

    orig_imu_x = np.array([x[0] - imu_start_s for x in imu_data])
    orig_imu_y = np.array([x[1] for x in imu_data])

    imu_interp = scipy.interpolate.interp1d(orig_imu_x, orig_imu_y)
    imu_x = np.arange(args.log_start, args.log_end, 1.0 / audio_rate)
    imu_y = np.array([imu_interp(x) for x in imu_x])
    scaled_imu_y = imu_y / imu_y.max()

    clip_audio_y = audio_data[int(args.video_start * audio_rate) : int(args.video_end * audio_rate)]

    audio_reference_time_s = 0.5 * (args.video_start + args.video_end)

    corr = scipy.signal.correlate(scaled_imu_y, clip_audio_y, mode='same')
    best_fit = np.argmax(corr ** 2)
    best_fit_s = best_fit / audio_rate

    offset_s = best_fit_s - audio_reference_time_s + args.log_start

    print("offset_s", offset_s)

    clip_audio_x = np.arange(0., len(clip_audio_y) / audio_rate, 1.0 / audio_rate) + args.video_start + offset_s

    sound_speed_in_air_mps = 343.0
    actual_offset_s = offset_s + args.fudge + args.distance / sound_speed_in_air_mps
    print("actual_offset_s", actual_offset_s)

    pylab.plot(clip_audio_x, clip_audio_y, label='audio')
    pylab.plot(imu_x, scaled_imu_y, linewidth=2, label='imu')
    pylab.legend()
    pylab.show()


if __name__ == '__main__':
    main()
