#include <SDL2/SDL.h>
#include <bits/stdc++.h>
#include <cmath>

using namespace std;
const int FPS = 60;
const int frameDelay = 1000 / FPS;

struct State {
  double theta;
  double omega;

  State(double theta = 0, double omega = 0) {
    this->theta = theta;
    this->omega = omega;
  }

  State derivative(double g, double l, double damping) {
    State ds;
    ds.theta = this->omega;
    ds.omega = -1 * (g / l) * sin(this->theta) - damping * this->omega;
    return ds;
  };

  State operator+(const State &other) {
    return State(this->theta + other.theta, this->omega + other.omega);
  }
  State operator*(const double val) {
    return State(this->theta * val, this->omega * val);
  }
};

void drawDisc(SDL_Renderer *render, int cx, int cy, int radius) {
  for (int dy = -radius; dy <= radius; dy++) {
    int dx = (int)sqrt(radius * radius - dy * dy);
    SDL_RenderDrawLine(render, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

State RK2(State &s, double g, double l, double damping, double timeStep) {
  State f1 = s.derivative(g, l, damping);
  State f2 = (s + f1 * timeStep).derivative(g, l, damping);
  State s_ = s + f2 * timeStep;
  return s_;
}

State RK4(State &s, double g, double l, double damping, double timeStep) {
  State f1 = s.derivative(g, l, damping);
  State f2 = (s + f1 * (timeStep / 2)).derivative(g, l, damping);
  State f3 = (s + f2 * (timeStep / 2)).derivative(g, l, damping);
  State f4 = (s + f3 * timeStep).derivative(g, l, damping);

  State s_ = s + (f1 + f2 * 2 + f3 * 2 + f4) * (timeStep / 6);
  return s_;
}

int main() {
  double g = 9.8;
  double l = 75;
  double damping = 0.01;
  State s(M_PI * (135.0 / 180.0), 0);

  double time = 0;
  double timeStep = 0.1;

  if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
    printf("Failed to init SDL, %s", SDL_GetError());
    return -1;
  }

  SDL_Window *window =
      SDL_CreateWindow("Pendulum", SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, 600, 400, SDL_WINDOW_SHOWN);
  SDL_Renderer *render =
      SDL_CreateRenderer(window, 0, SDL_RENDERER_ACCELERATED);

  bool quit = false;
  bool simulate = true;

  int prevTime;

  while (!quit) {
    prevTime = SDL_GetTicks();

    int mx, my;
    Uint32 mouse = SDL_GetMouseState(&mx, &my);

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_QUIT) {
        quit = true;
      }
      if (ev.type == SDL_KEYDOWN) {
        if (ev.key.keysym.sym == SDLK_q) {
          quit = true;
        }
      }
    }

    if (mouse & SDL_BUTTON(SDL_BUTTON_LEFT)) {
      simulate = false;
    } else {
      simulate = true;
    }

    if (simulate) {
      s = RK4(s, g, l, damping, timeStep);
      time += timeStep;
    } else {
      double bx = mx - 300.0;
      double by = my - 200.0;

      s.theta = atan2(bx, by);
      s.omega = 0;
    }

    int cx = 300 + (int)(l * sin((s.theta)));
    int cy = 200 + (int)(l * cos((s.theta)));

    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_SetRenderDrawColor(render, 255, 255, 255, 255);
    SDL_RenderDrawLine(render, 300, 200, cx, cy);
    SDL_SetRenderDrawColor(render, 255, 0, 0, 255);
    drawDisc(render, cx, cy, 5);
    SDL_RenderPresent(render);

    int curTime = SDL_GetTicks();
    if (frameDelay > (curTime - prevTime)) {
      SDL_Delay(frameDelay - (curTime - prevTime));
    }
  }

  SDL_Quit();
}
