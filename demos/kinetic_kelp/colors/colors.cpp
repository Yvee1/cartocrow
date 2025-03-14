#include "colors.h"

namespace cartocrow::CB {
Color light_blue(166,206,227);
Color blue(31,120,180);
Color light_green(178,223,138);
Color green(51,160,44);
Color light_red(251,154,153);
Color red(227,26,28);
Color light_orange(253,191,111);
Color orange(255,127,0);
Color light_purple(202,178,214);
Color purple(106,61,154);
std::vector<Color> lights({
    CB::light_blue, CB::light_red, CB::light_green, CB::light_purple, CB::light_orange
});
}

namespace cartocrow::tableau {
std::vector<Color> alternatingDarkLight({
    {0x4E79A7}, // dark blue
    {0xA0CBE8}, // light blue
    {0xF28E2B}, // dark orange
    {0xFFBE7D}, // light orange
    {0x59A14F}, // dark green
    {0x8CD17D}, // light green
    {0xB6992D}, // dark yellow
    {0xF1CE63}, // light yellow
    {0x499894}, // dark turquoise
    {0x86BCB6}, // light turquoise
    {0xE15759}, // (dark) red
    {0xFF9D9A}, // light red (pink)
    {0x79706E}, // dark gray
    {0xBAB0AC}, // light gray
    {0xD37295}, // dark magenta
    {0xFABFD2}, // light magenta
    {0xB07AA1}, // dark violet
    {0xD4A6C8}, // light violet
    {0x9D7660}, // dark brown
    {0xD7B5A6}, // light brown
});

std::vector<Color> firstLightThenDark({
    {0xA0CBE8}, // light blue
    {0xFFBE7D}, // light orange
    {0x8CD17D}, // light green
    {0xF1CE63}, // light yellow
    {0x86BCB6}, // light turquoise
    {0xFF9D9A}, // light red (pink)
    {0xBAB0AC}, // light gray
    {0xFABFD2}, // light magenta
    {0xD4A6C8}, // light violet
    {0xD7B5A6}, // light brown
    {0x4E79A7}, // dark blue
    {0xF28E2B}, // dark orange
    {0x59A14F}, // dark green
    {0xB6992D}, // dark yellow
    {0x499894}, // dark turquoise
    {0xE15759}, // (dark) red
    {0x79706E}, // dark gray
    {0xD37295}, // dark magenta
    {0xB07AA1}, // dark violet
    {0x9D7660}, // dark brown
});
}