#include <Novice.h>
#include <algorithm>
#include <cmath>

#ifdef _DEBUG
#include <imgui.h>
#endif

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";
const float kEpsilon = 1.0e-6f;

// 2次元ベクトル
struct Vector2 {
	float x;
	float y;
};

// ベクトルの加算
Vector2 operator+(const Vector2& lhs, const Vector2& rhs) {
	return {lhs.x + rhs.x, lhs.y + rhs.y};
}

// ベクトルの減算
Vector2 operator-(const Vector2& lhs, const Vector2& rhs) {
	return {lhs.x - rhs.x, lhs.y - rhs.y};
}

// ベクトルのスカラー倍
Vector2 operator*(float scalar, const Vector2& vector) {
	return {scalar * vector.x, scalar * vector.y};
}

// 数を右に書いたスカラー倍
Vector2 operator*(const Vector2& vector, float scalar) {
	return scalar * vector;
}

// ベクトルの長さ
float Length(const Vector2& vector) {
	return std::hypot(vector.x, vector.y);
}

// ベクトルの正規化
Vector2 Normalize(const Vector2& vector) {
	float length = Length(vector);
	if (length <= kEpsilon) {
		return {0.0f, 0.0f};
	}
	return (1.0f / length) * vector;
}

// ワールド座標での台の形
float f(float x) {
	return 0.125f * x * x * x + 0.25f * x * x;
}

// 手で微分した台の傾き f'(x) = 3x^2/8 + x/2
float fPrime(float x) {
	return 0.375f * x * x + 0.5f * x;
}

// 台とバイクのパラメータ（長さはm）
const float kMinX = 0.0f;
const float kMaxX = 2.0f;
const float kMoveSpeed = 1.0f;
const float kDeltaTime = 1.0f / 60.0f;
const float kPixelsPerMeter = 240.0f;
const Vector2 kScreenOrigin = {400.0f, 640.0f};
const int kRampSegments = 200;
const float kBodyHalfLength = 0.15f;
const int kWheelRadius = 10;

// バイクを台の範囲内で移動する
float MoveBike(float x, float input, float deltaTime) {
	return std::clamp(x + input * kMoveSpeed * deltaTime, kMinX, kMaxX);
}

// y上向きのワールド座標を、y下向きの画面座標へ変換する
Vector2 WorldToScreen(const Vector2& position) {
	return {kScreenOrigin.x + kPixelsPerMeter * position.x, kScreenOrigin.y - kPixelsPerMeter * position.y};
}

// 画面座標で線を描く
void DrawScreenLine(const Vector2& start, const Vector2& end, unsigned int color) {
	Novice::DrawLine(static_cast<int>(start.x), static_cast<int>(start.y),
		static_cast<int>(end.x), static_cast<int>(end.y), color);
}

// 台を細かい折れ線で描く
void DrawRamp() {
	Novice::DrawLine(40, static_cast<int>(kScreenOrigin.y), 1240, static_cast<int>(kScreenOrigin.y), 0x606060FF);
	for (int index = 0; index < kRampSegments; ++index) {
		float x0 = kMinX + (kMaxX - kMinX) * static_cast<float>(index) / static_cast<float>(kRampSegments);
		float x1 = kMinX + (kMaxX - kMinX) * static_cast<float>(index + 1) / static_cast<float>(kRampSegments);
		DrawScreenLine(WorldToScreen({x0, f(x0)}), WorldToScreen({x1, f(x1)}), WHITE);
	}
}

// 台の接線方向に車体と車輪を描く
void DrawBike(const Vector2& position, const Vector2& forward) {
	Vector2 back = WorldToScreen(position - kBodyHalfLength * forward);
	Vector2 front = WorldToScreen(position + kBodyHalfLength * forward);
	DrawScreenLine(back, front, RED);
	Novice::DrawEllipse(static_cast<int>(back.x), static_cast<int>(back.y),
		kWheelRadius, kWheelRadius, 0.0f, WHITE, kFillModeWireFrame);
	Novice::DrawEllipse(static_cast<int>(front.x), static_cast<int>(front.y),
		kWheelRadius, kWheelRadius, 0.0f, WHITE, kFillModeWireFrame);
}

// 現在の位置と傾きのUIをまとめて描画する
void DrawSlopeUI(float x, const Vector2& position, float slope, const Vector2& forward) {
#ifdef _DEBUG
	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(370.0f, 180.0f), ImGuiCond_Once);
	if (ImGui::Begin("MT4_02_00 - Slope")) {
		ImGui::Text("A: move left / D: move right");
		ImGui::Separator();
		ImGui::Text("x = %.3f m", x);
		ImGui::Text("f(x) = %.3f m", position.y);
		ImGui::Text("f'(x) = %.3f", slope);
		ImGui::Text("forward = (%.3f, %.3f)", forward.x, forward.y);
		ImGui::Text("World: +Y up / 1 m = 240 px");
	}
	ImGui::End();
#else
	Novice::ScreenPrintf(20, 20, "MT4_02_00 : Slope / A: left / D: right");
	Novice::ScreenPrintf(20, 46, "x = %.3f m / f(x) = %.3f m / f'(x) = %.3f", x, position.y, slope);
	Novice::ScreenPrintf(20, 72, "forward = (%.3f, %.3f) / World: +Y up / 1 m = 240 px", forward.x, forward.y);
#endif
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	// 課題の入力値を初期化する
	float x = 0.0f;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// A/Dの移動量を求め、台の範囲に収める
		float input = static_cast<float>(keys[DIK_D] != 0) - static_cast<float>(keys[DIK_A] != 0);
		x = MoveBike(x, input, kDeltaTime);

		// ワールド座標で位置と台に沿う向きを求める
		Vector2 position = {x, f(x)};
		float slope = fPrime(x);
		Vector2 forward = Normalize({1.0f, slope});

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 描画するときだけ画面座標へ変換する
		DrawRamp();
		DrawBike(position, forward);
		DrawSlopeUI(x, position, slope, forward);
		Novice::ScreenPrintf(20, 685, "A / D : move along the ramp   |   ESC : exit");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
