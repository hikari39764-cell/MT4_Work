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

// 2次ベジェ曲線上の位置を求める
Vector2 Bezier(const Vector2& p0, const Vector2& p1, const Vector2& p2, float t) {
	float oneMinusT = 1.0f - t;
	return (oneMinusT * oneMinusT) * p0 + (2.0f * oneMinusT * t) * p1 + (t * t) * p2;
}

// 手で微分した2次ベジェ曲線の接線ベクトルを求める
Vector2 BezierDerivative(const Vector2& p0, const Vector2& p1, const Vector2& p2, float t) {
	return (2.0f * (1.0f - t)) * (p1 - p0) + (2.0f * t) * (p2 - p1);
}

// 今回の制御点は画面座標（y下向き、単位はpx）
const Vector2 kP0 = {200.0f, 570.0f};
const Vector2 kInitialP1 = {800.0f, 70.0f};
const Vector2 kP2 = {1080.0f, 570.0f};
const float kMinP1X = 400.0f;
const float kMaxP1X = 880.0f;
const float kMinP1Y = 20.0f;
const float kMaxP1Y = 570.0f;
const int kRoadSegments = 200;
const float kDirectionLineLength = 120.0f;
const unsigned int kColorRoad = 0xB0D8FFFF;
const unsigned int kColorPoint = 0xFFD040FF;

// 入力値を課題の操作範囲に収める
void ClampCurveInputs(float& t, Vector2& p1) {
	t = std::clamp(t, 0.0f, 1.0f);
	p1.x = std::clamp(p1.x, kMinP1X, kMaxP1X);
	p1.y = std::clamp(p1.y, kMinP1Y, kMaxP1Y);
}

// 画面座標で線を描く
void DrawScreenLine(const Vector2& start, const Vector2& end, unsigned int color) {
	Novice::DrawLine(static_cast<int>(start.x), static_cast<int>(start.y),
		static_cast<int>(end.x), static_cast<int>(end.y), color);
}

// tを細かく区切って道を折れ線で描く
void DrawBezierRoad(const Vector2& p0, const Vector2& p1, const Vector2& p2) {
	for (int index = 0; index < kRoadSegments; ++index) {
		float t0 = static_cast<float>(index) / static_cast<float>(kRoadSegments);
		float t1 = static_cast<float>(index + 1) / static_cast<float>(kRoadSegments);
		DrawScreenLine(Bezier(p0, p1, p2, t0), Bezier(p0, p1, p2, t1), kColorRoad);
	}
}

// 曲線上の点から一定の長さの接線を描く
void DrawCurvePoint(const Vector2& position, const Vector2& lineEnd) {
	DrawScreenLine(position, lineEnd, RED);
	Novice::DrawEllipse(static_cast<int>(position.x), static_cast<int>(position.y),
		8, 8, 0.0f, kColorPoint, kFillModeSolid);
}

// 操作と計算結果のUIをまとめて描画する
void DrawCurveUI(float& t, Vector2& p1, const Vector2& position, const Vector2& derivative) {
#ifdef _DEBUG
	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(370.0f, 235.0f), ImGuiCond_Once);
	if (ImGui::Begin("MT4_02_01 - Bezier Tangent")) {
		ImGui::SliderFloat("t", &t, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::SliderFloat("p1.x", &p1.x, kMinP1X, kMaxP1X, "%.1f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::SliderFloat("p1.y", &p1.y, kMinP1Y, kMaxP1Y, "%.1f", ImGuiSliderFlags_AlwaysClamp);
		if (ImGui::Button("Reset")) {
			t = 0.5f;
			p1 = kInitialP1;
		}
		ImGui::Separator();
		ImGui::Text("B(t) = (%.1f, %.1f)", position.x, position.y);
		ImGui::Text("B'(t) = (%.1f, %.1f)", derivative.x, derivative.y);
		ImGui::Text("Tangent length = %.0f px", kDirectionLineLength);
		ImGui::Text("Screen coordinates: +Y down");
	}
	ImGui::End();
#else
	Novice::ScreenPrintf(20, 20, "MT4_02_01 : Bezier Tangent / t = %.3f / p1 = (%.1f, %.1f)", t, p1.x, p1.y);
	Novice::ScreenPrintf(20, 46, "B(t) = (%.1f, %.1f) / B'(t) = (%.1f, %.1f)", position.x, position.y, derivative.x, derivative.y);
	Novice::ScreenPrintf(20, 72, "Tangent length = %.0f px / Screen coordinates: +Y down", kDirectionLineLength);
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
	float t = 0.5f;
	Vector2 p1 = kInitialP1;

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

		// 操作範囲を守り、曲線上の位置と接線を求める
		ClampCurveInputs(t, p1);
		Vector2 position = Bezier(kP0, p1, kP2, t);
		Vector2 derivative = BezierDerivative(kP0, p1, kP2, t);
		Vector2 forward = Normalize(derivative);
		Vector2 lineEnd = position + kDirectionLineLength * forward;

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 画面座標のまま道と一定長の接線を描画する
		DrawBezierRoad(kP0, p1, kP2);
		DrawCurvePoint(position, lineEnd);
		DrawCurveUI(t, p1, position, derivative);
		Novice::ScreenPrintf(20, 685, "t / p1 : adjust in ImGui (Debug)   |   ESC : exit");

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
