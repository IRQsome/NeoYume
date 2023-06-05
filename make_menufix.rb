# encoding: utf-8
# frozen_string_literal: true

require 'chunky_png'

def to_tiles(path)
    raise unless block_given?
    png = ChunkyPNG::Image.from_file(path)
    raise unless png.width % 8 == 0
    raise unless png.height % 8 == 0
    data = String.new
    (png.width/8).times do |tile|
        tx = tile*8
        png.height.times do |y|
            4.times do |x|
                data << yield(png[x*2+tx,y]) + (yield(png[x*2+tx+1,y])<<4)
            end
        end
    end
    return data
end

File.open("MENUFIX.DAT","wb") do |f|
    File.read("stuff/FUNSCII.DAT").bytes do |byte|
        4.times{|x|f << (byte[x*2]*14 + byte[x*2+1]*224 + 17).chr}
    end
    f << to_tiles("stuff/logo.png"){|c|ChunkyPNG::Color.g(c)/17 + 1}

end