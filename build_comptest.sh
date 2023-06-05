../spin2cpp/build/flexspin -2 -E -H 376832 -O1,extrasmall,inline-single,loop-reduce,experimental,aggressive-mem -DFF_FS_TINY=1 -DFF_FS_READONLY=1 --charset=oem neoyume_upper.spin2
../spin2cpp/build/flexspin -2 -E -H 233472 -O1,extrasmall,inline-single,loop-reduce,remove-bss,experimental,~fcache --charset=oem neoyume_input.spin2
../spin2cpp/build/flexspin -2 -l neoyume_lower.spin2
