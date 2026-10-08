File.open("log.txt.bin","wb") do |f|
    File.readlines("log.txt",chomp:true).each do |line|
        if line =~ /packet enc \$([A-F0-9_]+)/
            val = $1.to_i(16)
            #puts $1,val
            f.write([val].pack('L<'))
        end
    end
end