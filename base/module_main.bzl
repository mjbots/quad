# -*- python -*-

# Copyright 2014-2018 Josh Pieper, jjp@pobox.com.
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

def module_main(name, prefix, cname, deps):
    native.genrule(
        name = "{}_main".format(name),
        outs = ["{}_main.cc".format(name)],
        cmd = """cat > $(location {name}_main.cc) << EOF
#include "{prefix}/{name}.h"

#include "base/module_main.h"

int main(int argc, char**argv) {{
        return mjmech::base::main<{cname}>(argc, argv);
}}
EOF
        """.format(name=name, cname=cname, prefix=prefix),
    )

    native.cc_binary(
        name = "{}".format(name),
        srcs = [
            "{}_main.cc".format(name),
            name + ".h",
        ],
        deps = deps + [
            "@boost//:filesystem",
            "@boost//:date_time",
        ],
    )
