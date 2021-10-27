#include <cstdio>
#include <cstdlib>
#include <list>
#include <cmath>
#include <ctime>
#include <windows.h>

#define abs(x) (x>-x?x:-x)

using namespace std;

class vec3;
class ball;
class physics;
class display;

char pixel[10] = { ' ','.','-',':','=','+','*','%','#','@' }; //픽셀로 사용될 ASCII 문자 (밝기 낮은순)

class vec3 {

public:

	double x, y, z;

	vec3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	vec3() {
		x = y = z = 0;
	}

	double norm() { //벡터 크기
		double ret = sqrt(x * x + y * y + z * z);
		return ret;
	}

	vec3 proj(vec3 projection) { //Proj_p(this)
		double len = (*this) ^ projection / projection.norm();
		vec3 dir = projection / projection.norm();
		return dir * len;
	}

	vec3 orthogonal(void) { //직교하는 임의의 벡터, |v|!=0 이라고 가정한다
		if (x != 0) return { -(y + z) / x ,1,1 };
		if (y != 0) return { 1,-(x + z) / y,1 };
		return { 1,1,-(x + y) / z };
	}

	vec3 resize(double size) { //벡터 길이 조절
		return (*this) * size / this->norm();
	}

	bool includes(vec3 op) { //op가 this로 bound되어 있는지 확인
		if (abs(op.x) < x && abs(op.y) < y && abs(op.z) < z) return true;
		return false;
	}

	vec3 operator+(vec3 op) {
		return { x + op.x,y + op.y,z + op.z };
	}
	vec3 operator-(vec3 op) {
		return { x - op.x,y - op.y,z - op.z };
	}
	vec3 operator*(double op) {
		return { x * op,y * op,z * op };
	}
	vec3 operator/(double op) {
		return { x / op,y / op,z / op };
	}
	void operator+=(vec3 op) {
		(*this) = (*this) + op;
	}
	vec3 operator%(vec3 op) { //cross product
		return { y * op.z - z * op.y,z * op.x - x * op.z,x * op.y - y * op.x };
	}
	double operator^(vec3 op) { //dot product
		return x * op.x + y * op.y + z * op.z;
	}

};

class ball {

public:

	vec3 centor, velocity, force;
	double radius, mass;

	ball(vec3 centor, double radius, double mass) {
		this->centor = centor;
		this->radius = radius;
		this->mass = mass;
	}

	void hit_wall(vec3 lim) { //벽과 충돌처리
		int dir[2] = { -1,1 };
		for (int i = 0; i < 2; i++) {
			int now = dir[i];
			if (now * centor.x + radius - lim.x > 0) {
				velocity.x *= -1;
				double error = now * centor.x + radius - lim.x;
				centor.x -= now * (error + 1e-5);
			}
			if (now * centor.y + radius - lim.y > 0) {
				velocity.y *= -1;
				double error = now * centor.y + radius - lim.y;
				centor.y -= now * (error + 1e-5);
			}
			if (now * centor.z + radius - lim.z > 0) {
				velocity.z *= -1;
				double error = now * centor.z + radius - lim.z;
				centor.z -= now * (error + 1e-5);
			}
		}
	}

};

void coll(ball* A, ball* B) { //오브젝트간의 충돌처리
	vec3 direct = B->centor - A->centor;
	if (direct.norm() >= A->radius + B->radius) return; //충돌판정
	vec3 v1 = A->velocity.proj(direct), v2 = B->velocity.proj(direct);
	double m1 = A->mass, m2 = B->mass;
	vec3 v1_n = v1 * (m1 - m2) / (m1 + m2) + v2 * (2 * m2) / (m1 + m2);
	vec3 v2_n = v1 * (2 * m1) / (m1 + m2) + v2 * (m2 - m1) / (m1 + m2);
	A->velocity = A->velocity - v1 + v1_n;
	B->velocity = B->velocity - v2 + v2_n; //충돌 후 속도 업데이트
	double error = (A->radius + B->radius - direct.norm()) / 2;
	A->centor = A->centor - direct.resize(error + 1e-5);
	B->centor = B->centor + direct.resize(error + 1e-5); //끼임 현상 방지를 위한 오차 보정
}

class physics {

public:

	list<ball> balls; //오브젝트 리스트
	vec3 lim = { 200,200,200 }; //-200<x<200, -200<y<200, -200<z<200
	double gravity = -100; 
	double delta_t = 1; //프레임 당 진행시킬 시간

	void set_delta_t(double in) { delta_t = in; };

	void run() {
		for (auto& ball : balls) { //오브젝트 이동, 벽과 충돌처리
			ball.velocity += ball.force * delta_t / ball.mass;
			ball.centor += ball.velocity * delta_t;
			ball.hit_wall(lim);
		}
		for (auto& ball1 : balls) { //오브젝트간 충돌처리
			for (auto& ball2 : balls) {
				if (&ball1 == &ball2) continue;
				coll(&ball1, &ball2);
			}
		}
	}

	void add_ball(vec3 centor, double radius, double mass) { //오브젝트 추가
		ball temp(centor, radius, mass);
		temp.velocity = { 0,0,0 };
		temp.force = { 0,0,gravity * mass };
		balls.push_back(temp);
	}

};

class display {

private:

	char proj_plane[60][120]; //투영면 (화면)
	bool changed[60][120]; //픽셀이 변했을 때만 출력
	physics* model; //출력할 모델
	vec3 camera = {180,0,100}, gaze = { -100,0,-30 }; //시점의 위치, 시선의 방향 (var)
	double L = 30; //시점과 투영면의 거리 (const)

	double speed = 5; //시점 이동속도
	double sensitivity = 3e-2; //시선 회전 감도

	struct plane { 
		vec3 orthogonal, p0;
	};

	plane wall[6]; //벽 평면에 수직한 벡터, 점

	struct key {
		short key_type;
		vec3 dir;
	};

	key camera_key[6] = { //키보드 입력에 대한 시점 이동방향
		{0x57,{0,speed,0}},
		{0x41,{-speed,0,0}},
		{0x53,{0,-speed,0}},
		{0x44,{speed,0,0}},
		{0x20,{0,0,speed}},
		{0x10,{0,0,-speed}}
	};


public:

	display(physics* in) {
		model = in;
		system(" mode  con lines=80   cols=120 ");
		disable_cursor();
		for (int i = 0; i < 60; i++)
			for (int j = 0; j < 120; j++) changed[i][j] = true;
		double lx = model->lim.x, ly = model->lim.y, lz = model->lim.z;
		wall[0] = { {-1,0,0},{lx,0,0}};
		wall[1] = { {1,0,0},{-lx,0,0}};
		wall[2] = { {0,-1,0},{0,ly,0}};
		wall[3] = { {0,1,0},{0,-ly,0}};
		wall[4] = { {0,0,-1},{0,0,lz}};
		wall[5] = { {0,0,1},{0,0,-lz}};
	}

	void render() { //화면 렌더링
		for (int i = 0; i < 60; i++) { 
			for (int j = 0; j < 120; j++) { //픽셀당 처리
				char last = proj_plane[i][j];
				vec3 w = { gaze.y, -gaze.x, 0 };
				vec3 h = { -gaze.x * gaze.z, -gaze.y * gaze.z,gaze.x * gaze.x + gaze.y * gaze.y };
				vec3 sight = gaze.resize(L) + h.resize(29.5 - i) + w.resize((j - 59.5)/2);
				
				bool on_ball = false; //시선이 오브젝트에 있는가?

				double min_dis = 1e9; //시점과 오브젝트 중심의 거리가 짧을수록 시선과 먼저 만난다

				for (auto& ball : this->model->balls) { //시선이 오브젝트와 만나는지 확인
					if ((ball.centor - camera).norm() > min_dis) continue;
					vec3 closest = (ball.centor - camera).proj(sight)-ball.centor+camera;
					double d = closest.norm();
					if (d < ball.radius) {
						min_dis = (ball.centor - camera).norm();
						int brightness = (int)(10*sqrt(ball.radius * ball.radius - d * d) / ball.radius-0.5); //시선이 오브젝트와 만나는 점에서의 밝기
						proj_plane[i][j] = pixel[brightness];
						on_ball = true;
					}
				}

				if (!on_ball) { //시선에 벽에 있음
					for (int k = 0; k < 6; k++) { 
						if (abs((wall[k].orthogonal^sight)) < 1e-2) continue; //시선이 벽에 평행인지 확인
						double t = ((wall[k].p0^wall[k].orthogonal)-(camera^wall[k].orthogonal)) / (sight^wall[k].orthogonal);
						if (t <= 0) continue;
						if (!model->lim.includes(camera + sight * (t - 1e-2))) continue;
						int brightness = (int)abs(4*(wall[k].orthogonal^sight) / (wall[k].orthogonal.norm() * sight.norm())); //시선과 벽면이 만나는 점에서의 밝기
						proj_plane[i][j] = pixel[brightness];
						break;
					}
				}

				if (last != proj_plane[i][j]) changed[i][j] = true; //픽셀 변화 체크
			}
		}
	}

	void show() { //변화가 있었던 픽셀만 다시 출력
		for (int i = 0; i < 60; i++) {
			for (int j = 0; j < 120; j++) {
				if (!changed[i][j]) continue;
				gotoxy(j, i);
				fprintf(stdout,"%c", proj_plane[i][j]);
				changed[i][j] = false;
			}
		}
	}

	void gotoxy(int x, int y) { //커서 이동
		COORD pos = { x,y };
		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), pos); 
	}

	void disable_cursor() //커서 숨기기
	{
		CONSOLE_CURSOR_INFO cursorInfo = { 0, };
		cursorInfo.dwSize = 1; 
		cursorInfo.bVisible = FALSE;
		SetConsoleCursorInfo(GetStdHandle(STD_OUTPUT_HANDLE), &cursorInfo);
	}

	void move_camera() { // W A S D Shift Space 시점 이동
		vec3 f = { gaze.x,gaze.y,0 };
		vec3 r = { gaze.y, -gaze.x, 0 };
		vec3 u = { 0,0,1 };
		for (auto& key_data : camera_key) {
			if (GetAsyncKeyState(key_data.key_type)) {
				if (model->lim.includes(camera + r.resize(key_data.dir.x) + f.resize(key_data.dir.y) + u.resize(key_data.dir.z)))
					camera += r.resize(key_data.dir.x) + f.resize(key_data.dir.y) + u.resize(key_data.dir.z);
			}
		}
	}

	void move_gaze() { //Up Down Left Right 시선 회전
		vec3 u = { 0,0,1 };
		vec3 d = { 0,0,-1 };
		vec3 axis = { gaze.y, -gaze.x, 0 };
		if (GetAsyncKeyState(VK_UP)) {
			if (gaze.proj(u).norm() * 1.8 < gaze.norm() || (gaze.proj(u) ^ u) < 0)
				gaze = gaze.proj(axis) * (cos(sensitivity / 100) - 1) + ((axis % gaze) * sin(sensitivity / 100)) + gaze;
		}
		if (GetAsyncKeyState(VK_DOWN)) {
			if (gaze.proj(d).norm() * 1.8 < gaze.norm() || (gaze.proj(d) ^ d) < 0)
				gaze = gaze.proj(axis) * (cos(-sensitivity / 100) - 1) + ((axis % gaze) * sin(-sensitivity / 100)) + gaze;
		}
		if (GetAsyncKeyState(VK_RIGHT)) {
			gaze = { gaze.x * cos(-sensitivity) - gaze.y * sin(-sensitivity),gaze.x * sin(-sensitivity) + gaze.y * cos(-sensitivity),gaze.z };
		}
		if (GetAsyncKeyState(VK_LEFT)) {
			gaze = { gaze.x * cos(sensitivity) - gaze.y * sin(sensitivity),gaze.x * sin(sensitivity) + gaze.y * cos(sensitivity),gaze.z };
		}
	}
};

int main() {

	physics model;
	display screen(&model);

	double framerate = 60;
	model.set_delta_t(1 / framerate);

	model.add_ball({ -3,-3,50 }, 30, 2); //오브젝트 추가
	model.add_ball({ 1,5,17 }, 25, 1);
	model.add_ball({ -30,50,60 }, 40, 3);
	model.add_ball({ -50,30,49 }, 30, 2);
	model.add_ball({ -70,20,-40 }, 40, 3);
	model.add_ball({ 100,-55,-30 }, 60, 5);
	while (true) {

		clock_t start = clock();

		model.run();
		screen.move_camera();
		screen.move_gaze();
		screen.render();
		screen.show();

		Sleep(1000 / framerate);

		clock_t end = clock(); //프레임 실행 시간 측정

		model.set_delta_t(((double)end - start) / 1000); //출력이 늦어진 만큼 진행 속도 보정

	}

}