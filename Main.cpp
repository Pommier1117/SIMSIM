#include <Siv3D.hpp>
#include <iostream>
#include <cmath>

using Real = double;

constexpr Real PI = 3.14159265358979323846;
constexpr Real dt = 1e-8;
constexpr Real  g = 9.80665;

template< typename Shape >
struct UniformlyAccelerated {
    Real vx, vy;
    Real ax, ay;

    UniformlyAccelerated(Real vx, Real vy, Real ax, Real ay)
        :vx(vx), vy(vy), ax(ax), ay(ay) {}

    auto go() {
        Shape* that = (Shape*)this;
        that->x += dt * that->vx;
        that->y += dt * that->vy;
        that->vx += dt * that->ax;
        that->vy += dt * that->ay;
    }
};

template< typename Shape >
struct Dragged {
    Real vx, vy;
    Real ax, ay;

    Dragged(Real vx, Real vy, Real ax, Real ay)
        :vx(vx), vy(vy), ax(ax), ay(ay) {}

    auto go() {
        Shape* that = (Shape*)this;
        Real v = std::sqrt(vx * vx + vy * vy);
        Real uvx = vx / v, uvy = vy / v;
        that->x += dt * that->vx;
        that->y += dt * that->vy;
        that->vx += dt * (that->ax - 0.5 * that->density_of_fluid * v * v * that->area * that->drag_coefficient * uvx / that->mass);
        that->vy += dt * (that->ay - 0.5 * that->density_of_fluid * v * v * that->area * that->drag_coefficient * uvy / that->mass);
    }
};

template< template< typename Shape > typename Motion >
struct Ball: Motion< Ball< Motion > > {
    Real drag_coefficient = 0.47, density_of_fluid = 1.293;
    Real radius, area, mass;
    Real x, y;

    template< typename... Ts >
    Ball(Real radius, Real mass, Real x, Real y, Ts&&... rest)
        : radius(radius), area(radius* radius* PI), mass(mass), x(x), y(y), Motion< Ball< Motion > >(rest...) {}
};

auto Main() {
    Camera2D camera(Vec2(0, 0), 1.0);
    Real log_timespeed = 0, timespeed = std::pow(10, log_timespeed);
    Real elapsed_time = 0, calculated_time = 0;

    Real radius = 0.01295,
        mass = 0.0005,
        theta = PI / 4,
        v0 = 1.82,
        x0 = 0,
        y0 = 0.153,
        vx = v0 * std::cos(theta),
        vy = v0 * std::sin(theta),
        ax = 0,
        ay = -g;
    Real t_max2, R;

    Ball< UniformlyAccelerated > b1(radius, mass, x0, y0, vx, vy, ax, ay);
    bool going = false, f1 = true, f2 = true;

    Window::Resize(1280, 720);
    Window::SetTitle(U"Dragged Motion");

    Print << U"theta = {} deg"_fmt(theta * 180 / PI);
    Print << U"y0 = {}"_fmt(y0);
    Print << U"v0 = {}"_fmt(v0);

    while (System::Update()) {
        camera.update(); {
            const auto tr1 = camera.createTransformer();
            const Transformer2D tr2(Mat3x2::Mat3x2(1, 0, 0, -1, 0, 0));

            Real xl = Scene::Width () / camera.getScale(),
                 yl = Scene::Height() / camera.getScale();

            Line(camera.getCenter().x - xl / 2, 0, camera.getCenter().x + xl / 2, 0).draw(1 / camera.getScale(), Palette::Skyblue);
            Line(0, -camera.getCenter().y - yl / 2, 0, -camera.getCenter().y + yl / 2).draw(1 / camera.getScale(), Palette::Skyblue);

            Circle(b1.x * 1000, b1.y * 1000, b1.radius * 1000).draw(Palette::Lightpink);
        }

        if (SimpleGUI::Button(U"START", Vec2(900, 100)))
            going = true;
        if (SimpleGUI::Button(U"PAUSE", Vec2(1100, 100)))
            going = false;

        SimpleGUI::Slider(U"{:>5}"_fmt(timespeed = std::pow(10, log_timespeed)), log_timespeed, -5, 5, Vec2(750, 40), 60, 400);
        camera.draw(Palette::White);

        if (!going) continue;
        elapsed_time += Scene::DeltaTime() * timespeed;

        while (elapsed_time > (calculated_time += dt)) {
            if (b1.y >= 0) b1.go();
            else if (f1) {
                f1 = false;
                Print << U"t\'_max2 = {}"_fmt(elapsed_time);
                Print << U"R\' = {}"_fmt(b1.x);
                Print << U"Delta t = {}"_fmt(elapsed_time - t_max2);
                Print << U"r = {}"_fmt(b1.x - R);
            }

            if (f2 && b1.y < y0) {
                f2 = false;
                Print << U"t_max2 = {}"_fmt(t_max2 = elapsed_time);
                Print << U"R = {}"_fmt(R = b1.x);
            }
        }
    }
}
