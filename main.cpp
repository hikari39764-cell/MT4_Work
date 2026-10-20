#include <Novice.h>
#include <cmath>

#ifdef _DEBUG
#include <imgui.h>
#endif

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";
const float kEpsilon = 1.0e-6f;
const int kRowHeight = 26;

// 四元数
struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};

// Quaternionの加算
Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
	return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w};
}

// Quaternionの符号反転
Quaternion operator-(const Quaternion& quaternion) {
	return {-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w};
}

// Quaternionのスカラー倍
Quaternion operator*(float scalar, const Quaternion& quaternion) {
	return {scalar * quaternion.x, scalar * quaternion.y, scalar * quaternion.z, scalar * quaternion.w};
}

// Quaternionの内積
float Dot(const Quaternion& lhs, const Quaternion& rhs) {
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

// Quaternionの積
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs) {
	return {
		lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
		lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
		lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
	};
}

// 乗法単位元を返す
Quaternion IdentityQuaternion() {
	return {0.0f, 0.0f, 0.0f, 1.0f};
}

// 共役Quaternionを返す
Quaternion Conjugate(const Quaternion& quaternion) {
	return {-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w};
}

// Quaternionのノルムを返す
float Norm(const Quaternion& quaternion) {
	return std::sqrt(Dot(quaternion, quaternion));
}

// 正規化したQuaternionを返す
Quaternion Normalize(const Quaternion& quaternion) {
	float norm = Norm(quaternion);
	// 零Quaternionは正規化できないため、無回転として扱う
	if (norm <= kEpsilon) {
		return IdentityQuaternion();
	}
	return (1.0f / norm) * quaternion;
}

// 逆Quaternionを返す
Quaternion Inverse(const Quaternion& quaternion) {
	float normSquared = Dot(quaternion, quaternion);
	// 零Quaternionに逆元はないため、無回転へフォールバックする
	if (normSquared <= kEpsilon * kEpsilon) {
		return IdentityQuaternion();
	}
	return (1.0f / normSquared) * Conjugate(quaternion);
}

// Quaternionを画面に表示する
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label) {
	Novice::ScreenPrintf(x, y, "%8.3f %8.3f %8.3f %8.3f : %s",
		quaternion.x, quaternion.y, quaternion.z, quaternion.w, label);
}

// 入力用UIをまとめて描画する
void DrawQuaternionControls(Quaternion& q1, Quaternion& q2) {
#ifdef _DEBUG
	ImGui::SetNextWindowPos(ImVec2(20.0f, 370.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(480.0f, 155.0f), ImGuiCond_Once);
	if (ImGui::Begin("Quaternion Controls")) {
		ImGui::DragFloat4("q1 (x,y,z,w)", &q1.x, 0.01f);
		ImGui::DragFloat4("q2 (x,y,z,w)", &q2.x, 0.01f);
		if (ImGui::Button("Reset")) {
			q1 = {2.0f, 3.0f, 4.0f, 1.0f};
			q2 = {1.0f, 3.0f, 5.0f, 2.0f};
		}
		ImGui::Text("Zero input: identity fallback (inverse is undefined).");
	}
	ImGui::End();
#else
	(void)q1;
	(void)q2;
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
	Quaternion q1 = {2.0f, 3.0f, 4.0f, 1.0f};
	Quaternion q2 = {1.0f, 3.0f, 5.0f, 2.0f};

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

		// Quaternionの各演算を求める
		Quaternion identity = IdentityQuaternion();
		Quaternion conj = Conjugate(q1);
		Quaternion inv = Inverse(q1);
		Quaternion normal = Normalize(q1);
		Quaternion mul1 = Multiply(q1, q2);
		Quaternion mul2 = Multiply(q2, q1);
		float norm = Norm(q1);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 課題の入力と実行結果を表示する
		Novice::ScreenPrintf(20, 20, "MT4_01_00 : Quaternion (x, y, z, w)");
		QuaternionScreenPrintf(20, 50, q1, "q1");
		QuaternionScreenPrintf(20, 50 + kRowHeight, q2, "q2");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 2, identity, "identity");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 3, conj, "conjugate(q1)");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 4, inv, "inverse(q1)");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 5, normal, "normalize(q1)");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 6, mul1, "q1 * q2");
		QuaternionScreenPrintf(20, 50 + kRowHeight * 7, mul2, "q2 * q1");
		Novice::ScreenPrintf(20, 50 + kRowHeight * 8, "Norm(q1) = %.6f", norm);
		DrawQuaternionControls(q1, q2);

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
