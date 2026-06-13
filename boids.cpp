#include <GL/freeglut.h>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <algorithm>

// Simulation constants
const int WIDTH = 800;
const int HEIGHT = 600;
const int NUM_BOIDS = 300; 
const float MAX_SPEED = 3.5f;
const float MAX_FORCE = 0.15f;
const float VIEW_RADIUS = 50.0f;
const float SEPARATION_RADIUS = 25.0f;
const float OBSTACLE_RADIUS = 40.0f;

// Grid constants
const int CELL_SIZE = (int)VIEW_RADIUS;
const int GRID_COLS = WIDTH / CELL_SIZE + 1;
const int GRID_ROWS = HEIGHT / CELL_SIZE + 1;

// Weights
const float SEPARATION_WEIGHT = 1.7f;
const float ALIGNMENT_WEIGHT = 1.0f;
const float COHESION_WEIGHT = 1.0f;
const float AVOIDANCE_WEIGHT = 4.0f;

bool showObstacles = true;

struct Vector2 {
    float x, y;
    Vector2(float x = 0, float y = 0) : x(x), y(y) {}
    Vector2 operator+(const Vector2& v) const { return Vector2(x + v.x, y + v.y); }
    Vector2 operator-(const Vector2& v) const { return Vector2(x - v.x, y - v.y); }
    Vector2 operator*(float s) const { return Vector2(x * s, y * s); }
    Vector2 operator/(float s) const { return Vector2(x / s, y / s); }
    Vector2& operator+=(const Vector2& v) { x += v.x; y += v.y; return *this; }
    Vector2& operator-=(const Vector2& v) { x -= v.x; y -= v.y; return *this; }
    float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }
    Vector2 normalize() const {
        float l = length();
        return (l > 0) ? *this / l : Vector2(0, 0);
    }
    Vector2 limit(float max) const {
        return (lengthSq() > max * max) ? normalize() * max : *this;
    }
};

struct Obstacle {
    Vector2 pos;
    float radius;
    bool isBeingDragged;

    Obstacle(float x, float y, float r) : pos(x, y), radius(r), isBeingDragged(false) {}

    void draw() {
        if (!showObstacles) return;
        // Draw glow/shadow
        glBegin(GL_TRIANGLE_FAN);
        glColor4f(0.0f, 0.0f, 0.0f, 0.3f);
        glVertex2f(pos.x, pos.y);
        glColor4f(0.0f, 0.0f, 0.0f, 0.0f);
        for(int i=0; i<=32; i++) {
            float theta = 2.0f * M_PI * float(i) / 32.0f;
            glVertex2f(pos.x + (radius + 20.0f) * cosf(theta), pos.y + (radius + 20.0f) * sinf(theta));
        }
        glEnd();

        // Draw main body (Stone-like)
        glBegin(GL_POLYGON);
        if (isBeingDragged) glColor4f(0.3f, 0.3f, 0.35f, 0.9f);
        else glColor4f(0.2f, 0.2f, 0.25f, 0.8f);
        
        for(int i=0; i<32; i++) {
            float theta = 2.0f * M_PI * float(i) / 32.0f;
            float r = radius * (0.95f + 0.05f * sinf(theta * 5.0f));
            glVertex2f(pos.x + r * cosf(theta), pos.y + r * sinf(theta));
        }
        glEnd();
    }

    bool contains(float px, float py) {
        if (!showObstacles) return false;
        float dx = pos.x - px;
        float dy = pos.y - py;
        return (dx*dx + dy*dy) < radius*radius;
    }
};

class Boid {
public:
    Vector2 pos, vel, acc;
    int id;

    Boid(int id, float x, float y) : id(id), pos(x, y) {
        float angle = (float)rand() / RAND_MAX * 2.0f * M_PI;
        vel = Vector2(std::cos(angle), std::sin(angle)) * MAX_SPEED;
    }

    void applyForce(Vector2 force) { acc += force; }

    void update() {
        vel = (vel + acc).limit(MAX_SPEED);
        pos += vel;
        acc = Vector2(0, 0);
        if (pos.x < 0) pos.x += WIDTH; if (pos.x > WIDTH) pos.x -= WIDTH;
        if (pos.y < 0) pos.y += HEIGHT; if (pos.y > HEIGHT) pos.y -= HEIGHT;
    }

    Vector2 avoidObstacles(const std::vector<Obstacle>& obstacles) {
        if (!showObstacles) return Vector2(0, 0);
        Vector2 steer(0, 0);
        for (const auto& obs : obstacles) {
            Vector2 toObstacle = obs.pos - pos;
            float dist = toObstacle.length();
            if (dist < obs.radius + VIEW_RADIUS) {
                steer += (pos - obs.pos).normalize() * (MAX_SPEED * 3.0f / (dist + 1.0f));
                float dot = vel.normalize().x * toObstacle.normalize().x + vel.normalize().y * toObstacle.normalize().y;
                if (dot > 0.7f) {
                    Vector2 lateral(-vel.y, vel.x);
                    steer += lateral.normalize() * MAX_SPEED;
                }
            }
        }
        return steer.limit(MAX_FORCE * 2.5f);
    }

    void flock(const std::vector<Boid*>& neighbors, const std::vector<Obstacle>& obstacles) {
        Vector2 sep, ali, coh;
        int sCount = 0, aCount = 0, cCount = 0;
        
        for (Boid* other : neighbors) {
            if (other->id == id) continue;
            float d = (pos - other->pos).length();
            if (d > 0 && d < SEPARATION_RADIUS) {
                sep += (pos - other->pos).normalize() / d;
                sCount++;
            }
            if (d > 0 && d < VIEW_RADIUS) {
                ali += other->vel; aCount++;
                coh += other->pos; cCount++;
            }
        }

        if (sCount > 0) sep = (sep / (float)sCount).normalize() * MAX_SPEED - vel;
        if (aCount > 0) ali = (ali / (float)aCount).normalize() * MAX_SPEED - vel;
        if (cCount > 0) coh = ((coh / (float)cCount) - pos).normalize() * MAX_SPEED - vel;

        applyForce(sep.limit(MAX_FORCE) * SEPARATION_WEIGHT);
        applyForce(ali.limit(MAX_FORCE) * ALIGNMENT_WEIGHT);
        applyForce(coh.limit(MAX_FORCE) * COHESION_WEIGHT);
        applyForce(avoidObstacles(obstacles) * AVOIDANCE_WEIGHT);
    }
};

struct Cloud {
    Vector2 pos;
    float scale;
    Cloud(float x, float y, float s) : pos(x, y), scale(s) {}
    void draw() {
        glColor4f(1.0f, 0.8f, 0.7f, 0.3f); // Sunset-tinted clouds
        for(int i = 0; i < 6; i++) {
            float offX = (i % 3) * 25.0f * scale;
            float offY = (i / 3) * 20.0f * scale;
            float r = (30.0f + (rand() % 10)) * scale;
            glBegin(GL_POLYGON);
            for(int j=0; j<20; j++) {
                float theta = 2.0f * M_PI * float(j) / 20.0f;
                glVertex2f(pos.x + offX + r * cosf(theta), pos.y + offY + r * sinf(theta));
            }
            glEnd();
        }
    }
};

std::vector<Boid> boids;
std::vector<Obstacle> obstacles;
std::vector<Cloud> clouds;
std::vector<int> grid[GRID_COLS][GRID_ROWS];

void updateGrid() {
    for (int i = 0; i < GRID_COLS; ++i)
        for (int j = 0; j < GRID_ROWS; ++j)
            grid[i][j].clear();

    for (int i = 0; i < (int)boids.size(); ++i) {
        int gx = std::max(0, std::min(GRID_COLS - 1, (int)(boids[i].pos.x / CELL_SIZE)));
        int gy = std::max(0, std::min(GRID_ROWS - 1, (int)(boids[i].pos.y / CELL_SIZE)));
        grid[gx][gy].push_back(i);
    }
}

void drawBackground() {
    glBegin(GL_QUADS);
    // Evening gradient: Dark purple to Sunset orange
    glColor3f(0.15f, 0.1f, 0.3f); // Top
    glVertex2f(0, HEIGHT);
    glVertex2f(WIDTH, HEIGHT);
    glColor3f(0.9f, 0.4f, 0.2f); // Bottom
    glVertex2f(WIDTH, 0);
    glVertex2f(0, 0);
    glEnd();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    
    drawBackground();
    for (auto& c : clouds) c.draw();
    for (auto& o : obstacles) o.draw();

    updateGrid();

    for (auto& b : boids) {
        std::vector<Boid*> neighbors;
        int gx = (int)(b.pos.x / CELL_SIZE);
        int gy = (int)(b.pos.y / CELL_SIZE);
        for (int nx = gx - 1; nx <= gx + 1; ++nx) {
            for (int ny = gy - 1; ny <= gy + 1; ++ny) {
                int cx = (nx + GRID_COLS) % GRID_COLS;
                int cy = (ny + GRID_ROWS) % GRID_ROWS;
                for (int idx : grid[cx][cy]) neighbors.push_back(&boids[idx]);
            }
        }
        b.flock(neighbors, obstacles);
        b.update();
        
        float angle = std::atan2(b.vel.y, b.vel.x);
        glPushMatrix();
        glTranslatef(b.pos.x, b.pos.y, 0);
        glRotatef(angle * 180.0f / M_PI, 0, 0, 1);
        glBegin(GL_TRIANGLES);
        glColor3f(0.0f, 0.0f, 0.0f); // Black boids
        glVertex2f(12, 0); glVertex2f(-6, 4); glVertex2f(-6, -4);
        glEnd();
        glPopMatrix();
    }
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y) {
    float mx = (float)x;
    float my = (float)(HEIGHT - y);

    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            for (auto& o : obstacles) {
                if (o.contains(mx, my)) {
                    o.isBeingDragged = true;
                    break;
                }
            }
        } else if (state == GLUT_UP) {
            for (auto& o : obstacles) o.isBeingDragged = false;
        }
    }
}

void motion(int x, int y) {
    float mx = (float)x;
    float my = (float)(HEIGHT - y);
    for (auto& o : obstacles) {
        if (o.isBeingDragged) {
            o.pos.x = mx;
            o.pos.y = my;
        }
    }
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 'h' || key == 'H') {
        showObstacles = !showObstacles;
    }
}

void timer(int value) {
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

void init() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WIDTH, 0, HEIGHT);
    glMatrixMode(GL_MODELVIEW);

    std::srand(std::time(0));
    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids.emplace_back(i, (float)(rand() % WIDTH), (float)(rand() % HEIGHT));
    }

    for (int i = 0; i < 4; ++i) {
        obstacles.emplace_back((float)(100 + rand() % (WIDTH - 200)), 
                               (float)(100 + rand() % (HEIGHT - 200)), 
                               OBSTACLE_RADIUS);
    }

    for (int i = 0; i < 8; ++i) {
        clouds.emplace_back((float)(rand() % WIDTH), (float)(rand() % HEIGHT), 0.6f + (rand() % 100) / 100.0f);
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Boids Evening Sky");
    init();
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(0, timer, 0);
    glutMainLoop();
    return 0;
}
