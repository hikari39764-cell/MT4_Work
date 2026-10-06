#include <Novice.h>
#include <algorithm>
#include <cmath>
#include <numbers>

#ifdef _DEBUG
#include <imgui.h>
#endif

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

// 3次元ベクトル
struct Vector3 {
	float x;
	float y;
	float z;
};

// 4x4行列
struct Matrix4x4 {
	float m[4][4];
};

// 球面座標
struct Spherical {
	float radius; // 動径
	float theta;  // 仰角
	float phi;    // 方位角
};

// 球面座標から直交座標へ変換する
Vector3 ToCartesian(const Spherical& s) {
	float rho = s.radius * std::cos(s.theta);
	return {rho * std::cos(s.phi), s.radius * std::sin(s.theta), rho * std::sin(s.phi)};
}

// ベクトルを正規化する
Vector3 Normalize(const Vector3& v) {
	float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (length == 0.0f) {
		return {0.0f, 0.0f, 0.0f};
	}
	return {v.x / length, v.y / length, v.z / length};
}

// 外積を求める
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x,
	};
}

// 注視点を向くカメラのワールド行列を作成する
Matrix4x4 MakeCameraMatrix(const Vector3& eye, const Vector3& target) {
	Vector3 worldUp = {0.0f, 1.0f, 0.0f};
	Vector3 forward = Normalize({target.x - eye.x, target.y - eye.y, target.z - eye.z});
	Vector3 right = Normalize(Cross(worldUp, forward));
	Vector3 up = Cross(forward, right);

	return {{
		{right.x, right.y, right.z, 0.0f},
		{up.x, up.y, up.z, 0.0f},
		{forward.x, forward.y, forward.z, 0.0f},
		{eye.x, eye.y, eye.z, 1.0f},
	}};
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	// 球面座標と注視点の初期化
	const float halfPi = std::numbers::pi_v<float> / 2.0f;
	const float limit = halfPi - 0.01f;
	Spherical s = {6.0f, 0.0f, -halfPi};
	Vector3 target = {0.0f, 0.0f, 0.0f};

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

#ifdef _DEBUG
		// 球面座標を編集する
		ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(580.0f, 320.0f), ImGuiCond_Once);
		bool isWindowOpen = ImGui::Begin("Spherical Coordinates");
		if (isWindowOpen) {
			ImGui::Text("Target: (0, 0, 0) / +Y up / Camera +Z forward");
			ImGui::Separator();
			ImGui::InputFloat("Radius", &s.radius, 0.1f, 1.0f, "%.3f");
			ImGui::InputFloat("Theta: elevation (rad)", &s.theta, 0.01f, 0.1f, "%.3f");
			ImGui::InputFloat("Phi: azimuth (rad)", &s.phi, 0.01f, 0.1f, "%.3f");
		}
#endif

		// 注視点との重なりと真上・真下を避ける
		s.radius = (std::max)(s.radius, 0.1f);
		s.theta = std::clamp(s.theta, -limit, limit);

		// カメラの位置とワールド行列を求める
		Vector3 offset = ToCartesian(s);
		Vector3 pos = {target.x + offset.x, target.y + offset.y, target.z + offset.z};
		Matrix4x4 cameraMatrix = MakeCameraMatrix(pos, target);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

#ifdef _DEBUG
		// 球面座標と直交座標、カメラ行列を表示する
		if (isWindowOpen) {
			ImGui::Separator();
			ImGui::Text("Spherical: r = %.3f, theta = %.3f rad, phi = %.3f rad", s.radius, s.theta, s.phi);
			ImGui::Text("Cartesian: x = %.3f, y = %.3f, z = %.3f", pos.x, pos.y, pos.z);
			ImGui::Separator();
			ImGui::Text("Camera matrix");
			for (int row = 0; row < 4; ++row) {
				ImGui::Text("%8.3f %8.3f %8.3f %8.3f",
					cameraMatrix.m[row][0], cameraMatrix.m[row][1], cameraMatrix.m[row][2], cameraMatrix.m[row][3]);
			}
		}
		ImGui::End();
#else
		// Releaseでは計算結果を画面に表示する
		Novice::ScreenPrintf(20, 20, "Spherical: r = %.3f, theta = %.3f rad, phi = %.3f rad", s.radius, s.theta, s.phi);
		Novice::ScreenPrintf(20, 40, "Cartesian: x = %.3f, y = %.3f, z = %.3f", pos.x, pos.y, pos.z);
		Novice::ScreenPrintf(20, 60, "Camera matrix");
		for (int row = 0; row < 4; ++row) {
			Novice::ScreenPrintf(20, 80 + row * 20, "%8.3f %8.3f %8.3f %8.3f",
				cameraMatrix.m[row][0], cameraMatrix.m[row][1], cameraMatrix.m[row][2], cameraMatrix.m[row][3]);
		}
#endif

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
