set -e # Enable error reporting
rm neoyume_lower.binary || true
flexspin -2 -E -H 376832 -O1,extrasmall,inline-single,loop-reduce --charset=oem -DFF_FS_TINY=1 -DFF_FS_READONLY=1 neoyume_upper.spin2
flexspin -2 -E -H 233472 -O1,extrasmall,inline-single,loop-reduce,~fcache --charset=oem neoyume_input.spin2
flexspin -2 -l --compress neoyume_lower.spin2
