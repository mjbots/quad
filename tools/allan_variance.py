#!/usr/bin/env python

# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

'''%prog [options]

Calculate Allan Variance statistics for a record in a given .tlog
file.
'''

import optparse
import os
import sys

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, '../python'))

import telemetry_log


class VarianceOld(object):
    # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0

    def add(self, value):
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.M2 += delta * (value - self.mean)

    def variance(self):
        return self.M2 / (self.n - 1)


class Variance(object):
    def __init__(self):
        self.n = 0
        self.M2 = 0.0

    def add(self, value):
        self.n += 1
        self.M2 += value ** 2

    def variance(self):
        return self.M2 / self.n

    def stddev(self):
        return self.variance() ** .5


class ChannelWindow(object):
    def __init__(self, window_s):
        self.window_s = window_s
        self.variance = Variance()

        self.last_value = None
        self.current_value = 0.0
        self.current_count = 0
        self.current_time_s = None

    def valid(self):
        return self.variance.n >= 2

    def add(self, timestamp_s, value):
        if self.current_time_s is None:
            self.current_time_s = timestamp_s

        if timestamp_s - self.current_time_s > self.window_s:
            new_value = self.current_value / self.current_count

            if self.last_value:
                self.variance.add(new_value - self.last_value)
            self.last_value = new_value

            while True:
                self.current_time_s += self.window_s
                if (self.current_time_s + self.window_s) >= timestamp_s:
                    break
                self.last_value = None

            self.current_value = 0
            self.current_count = 0

        self.current_value += value
        self.current_count += 1


class Channel(object):
    def __init__(self, periods):
        self.windows = [ChannelWindow(x) for x in periods]

    def add(self, timestamp_s, value):
        for w in self.windows:
            w.add(timestamp_s, value)


def enumerate_fields(sample, predicate):
    result = []
    for name, value in sample.__dict__.iteritems():
        if predicate(name, value):
            result.append(name)
        elif hasattr(value, '__dict__'):
            result += [name + '.' + x for x in
                       enumerate_fields(value, predicate)]
    return result


def get(sample, name):
    if '.' in name:
        mine, child = name.split('.', 1)
        return get(getattr(sample, mine), child)
    return getattr(sample, name)


class Record(object):
    def __init__(self, sample):
        # Find the names of all structure elements which have a
        # floating point value and aren't labeled as a "timestamp" or
        # "time" of some sort.
        self.channels = {}
        self.periods = self.window_periods()
        self.timestamp_field = None
        self.timestamp_scale = 1.0

        enumerator = enumerate_fields(
            sample, lambda name, value: type(value) == float)
        for name in enumerator:
            self.channels[name] = Channel(self.periods)

        for name in enumerate_fields(sample,
                                     lambda name, value: 'timestamp' in name):
            self.timestamp_scale = 1e-4
            self.timestamp_field = name
            break

    def window_periods(self):
        period_s = 0.05
        result = []
        while period_s < 1e7:
            result.append(period_s)
            period_s *= 1.05

        return result

    def valid_periods_indices(self):
        result = []
        name = self.channels.keys()[0]
        for i in range(len(self.periods)):
            if not self.channels[name].windows[i].valid():
                break
            result.append(i)
        return result

    def add(self, sample):
        timestamp_s = get(sample, self.timestamp_field) * self.timestamp_scale
        for key, channel in self.channels.iteritems():
            value = get(sample, key)
            channel.add(timestamp_s, value)


def main():
    usage, description = __doc__.split('\n\n', 1)
    parser = optparse.OptionParser(usage=usage, description=description)

    parser.add_option('--input', '-i', help='tlog file')
    parser.add_option('--record', '-r', help='record to measure')
    parser.add_option('--limit', '-l', type='int', default=None,
                      help='maximum records to read')

    options, args = parser.parse_args()
    assert len(args) == 0, 'unexpected arguments'

    assert options.input is not None
    br = telemetry_log.BulkReader(open(options.input))
    record = None

    i = 0
    for name, data in br.items([options.record]):
        i += 1
        if options.limit and i > options.limit:
            break

        if record is None:
            record = Record(data)
            if len(record.channels) == 0:
                raise RuntimeError('no float channels found')
        record.add(data)


    # Do the header.
    print 'Time',
    for name in sorted(record.channels.keys()):
        print ',', name,
    print

    for period_index in record.valid_periods_indices():
        print record.periods[period_index],

        for name in sorted(record.channels.keys()):
            channel = record.channels[name]
            print ',', channel.windows[period_index].variance.stddev(),

        print


if __name__ == '__main__':
    main()

