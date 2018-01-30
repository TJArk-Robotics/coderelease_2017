#ifndef COLOR_H
#define COLOR_H


struct color{
    int cy;
    int cb;
    int cr;
    color() = default;
    color(int _cy, int _cb, int _cr) : cy(_cy), cb(_cb), cr(_cr) {}
    color(const color&) = default;
    color(color&&) = default;
    color& operator=(const color&) = default;
    color& operator=(color&&) = default;
};


#endif 

