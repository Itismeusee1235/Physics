#include <SDL2/SDL.h>
#include <bits/stdc++.h>

using namespace std;
const int FPS = 60;
const int frameDelay = 1000 / FPS;

struct Point {
  int x;
  int y;
  Point(int x, int y) {
    this->x = x;
    this->y = y;
  };
};

struct State {
  double t;
  double theta1;
  double theta2;
  double omega1;
  double omega2;

  State(double theta1 = 0, double theta2 = 0, double omega1 = 0,
        double omega2 = 0) {
    this->theta1 = theta1;
    this->theta2 = theta2;
    this->omega1 = omega1;
    this->omega2 = omega2;
  }

  State derivative(double g, double m1, double m2, double l1, double l2,
                   double damping) {
    State s_;
    s_.theta1 = omega1;
    s_.theta2 = omega2;
    s_.omega1 = -g * (2 * m1 + m2) * sin(theta1) -
                m2 * g * sin(theta1 - 2 * theta2) -
                2 * sin(theta1 - theta2) * m2 *
                    (omega2 * omega2 * l2 +
                     omega1 * omega1 * l1 * cos(theta1 - theta2));
    s_.omega1 /= l1 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));

    s_.omega2 =
        2 * sin(theta1 - theta2) *
        (omega1 * omega1 * l1 * (m1 + m2) + g * (m1 + m2) * cos(theta1) +
         omega2 * omega2 * l2 * m2 * cos(theta1 - theta2));
    s_.omega2 /= l2 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));

    return s_;
  }

  State operator+(const State &other) {
    return State(other.theta1 + theta1, other.theta2 + theta2,
                 other.omega1 + omega1, other.omega2 + omega2);
  }

  State operator*(const double val) {
    return State(val * theta1, val * theta2, val * omega1, val * omega2);
  }
};

State RK4(State &s, double g, double l1, double l2, double m1, double m2,
          double damping, double timeStep) {
  State f1 = s.derivative(g, m1, m2, l1, l2, damping);
  State f2 = (s + f1 * (timeStep / 2)).derivative(g, m1, m2, l1, l2, damping);
  State f3 = (s + f2 * (timeStep / 2)).derivative(g, m1, m2, l1, l2, damping);
  State f4 = (s + f3 * timeStep).derivative(g, m1, m2, l1, l2, damping);

  State s_ = s + (f1 + f2 * 2 + f3 * 2 + f4) * (timeStep / 6);
  return s_;
}

void drawDisc(SDL_Renderer *render, int cx, int cy, int radius) {
  for (int dy = -radius; dy <= radius; dy++) {
    int dx = (int)sqrt(radius * radius - dy * dy);
    SDL_RenderDrawLine(render, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}

void drawPoints(SDL_Renderer *render, vector<SDL_Point> p, int r, int g, int b,
                bool fade) {
  int size = p.size();
  for (int i = size - 1; i >= 0; i--) {
    float fac = (i * 1.0) / size;
    if (!fade) {
      fac = 1;
    }
    SDL_SetRenderDrawColor(render, r * fac, g * fac, b * fac, 255);
    drawDisc(render, p[i].x, p[i].y, 1);
    // SDL_RenderDrawPoint(render, p[i].x, p[i].y);
  }
}

int main() {

  bool fade = true;
  bool dissaper = true;

  double g = 9.8;
  double m1 = 1;
  double m2 = 1;
  double l1 = 100;
  double l2 = 100;
  int maxPoints = 1024;

  int hx = 300;
  int hy = 100;

  double timeStep = 0.1;

  State s(M_PI / 2, -M_PI / 2.1, 0, 0);

  if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
    printf("Failed to init SDL, %s", SDL_GetError());
    return -1;
  }

  SDL_Window *windows =
      SDL_CreateWindow("Double Pendulum", SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, 600, 400, SDL_WINDOW_SHOWN);
  SDL_Renderer *renderer =
      SDL_CreateRenderer(windows, 0, SDL_RENDERER_ACCELERATED);

  bool quit = false;
  bool simulate = false;
  double prevTime = 0;

  vector<SDL_Point> p1;
  vector<SDL_Point> p2;

  while (!quit) {

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_QUIT) {
        quit = true;
      } else if (ev.type == SDL_KEYDOWN) {
        if (ev.key.keysym.sym == SDLK_q) {
          quit = true;
        } else if (ev.key.keysym.sym == SDLK_SPACE) {
          simulate = !simulate;
        }
      }
    }

    int c1x, c1y, c2x, c2y;
    if (simulate) {
      s = RK4(s, g, l1, l2, m1, m2, 0, timeStep);
      c1x = hx + (int)(l1 * sin(s.theta1));
      c1y = hy + (int)(l1 * cos(s.theta1));

      c2x = c1x + (int)(l2 * sin(s.theta2));
      c2y = c1y + (int)(l2 * cos(s.theta2));

      p1.push_back({c1x, c1y});
      p2.push_back({c2x, c2y});
    } else {
      c1x = hx + (int)(l1 * sin(s.theta1));
      c1y = hy + (int)(l1 * cos(s.theta1));

      c2x = c1x + (int)(l2 * sin(s.theta2));
      c2y = c1y + (int)(l2 * cos(s.theta2));
    }

    if (p1.size() > maxPoints && dissaper) {
      p1.erase(p1.begin());
    }
    if (p2.size() > maxPoints && dissaper) {
      p2.erase(p2.begin());
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    drawDisc(renderer, c1x, c1y, 5);
    drawPoints(renderer, p1, 255, 0, 0, fade);

    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    drawDisc(renderer, c2x, c2y, 5);
    drawPoints(renderer, p2, 0, 0, 255, fade);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer, hx, hy, c1x, c1y);
    SDL_RenderDrawLine(renderer, c1x, c1y, c2x, c2y);

    SDL_RenderPresent(renderer);

    double currTime = SDL_GetTicks();
    double diff = currTime - prevTime;
    if (frameDelay > diff) {
      SDL_Delay(frameDelay - diff);
    }
    prevTime = currTime;
  }
  SDL_Quit();

  return 0;
}
