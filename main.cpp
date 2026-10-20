#include <Novice.h>
#include <algorithm>
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

// ベクトルの加算
Vector3 operator+(const Vector3& lhs, const Vector3& rhs) {
	return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

// ベクトルの減算
Vector3 operator-(const Vector3& lhs, const Vector3& rhs) {
	return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

// ベクトルの符号反転
Vector3 operator-(const Vector3& vector) {
	return {-vector.x, -vector.y, -vector.z};
}

// ベクトルのスカラー倍
Vector3 operator*(float scalar, const Vector3& vector) {
	return {scalar * vector.x, scalar * vector.y, scalar * vector.z};
}

// ベクトルの内積
float Dot(const Vector3& lhs, const Vector3& rhs) {
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

// ベクトルの外積
Vector3 Cross(const Vector3& lhs, const Vector3& rhs) {
	return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

// ベクトルの長さ
float Length(const Vector3& vector) {
	return std::sqrt(Dot(vector, vector));
}

// ベクトルの正規化
Vector3 Normalize(const Vector3& vector) {
	float length = Length(vector);
	if (length <= kEpsilon) {
		return {0.0f, 0.0f, 0.0f};
	}
	return (1.0f / length) * vector;
}

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

// 任意軸回転を表すQuaternionを生成する
Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle) {
	Vector3 unitAxis = Normalize(axis);
	// 回転軸が定まらない場合は無回転とする
	if (Dot(unitAxis, unitAxis) == 0.0f) {
		return IdentityQuaternion();
	}
	float sinHalfAngle = std::sin(angle * 0.5f);
	return {unitAxis.x * sinHalfAngle, unitAxis.y * sinHalfAngle, unitAxis.z * sinHalfAngle, std::cos(angle * 0.5f)};
}

// Quaternionでベクトルを回転させる
Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion) {
	Quaternion rotation = Normalize(quaternion);
	Quaternion point = {vector.x, vector.y, vector.z, 0.0f};
	Quaternion result = Multiply(Multiply(rotation, point), Conjugate(rotation));
	return {result.x, result.y, result.z};
}

// Quaternionから回転行列を求める
Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion) {
	Quaternion q = Normalize(quaternion);
	// 行ベクトルv * Rを使い、q * v * conjugate(q)と回転方向をそろえる
	return {{
		{1.0f - 2.0f * (q.y * q.y + q.z * q.z), 2.0f * (q.x * q.y + q.z * q.w), 2.0f * (q.x * q.z - q.y * q.w), 0.0f},
		{2.0f * (q.x * q.y - q.z * q.w), 1.0f - 2.0f * (q.x * q.x + q.z * q.z), 2.0f * (q.y * q.z + q.x * q.w), 0.0f},
		{2.0f * (q.x * q.z + q.y * q.w), 2.0f * (q.y * q.z - q.x * q.w), 1.0f - 2.0f * (q.x * q.x + q.y * q.y), 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f},
	}};
}

// 行ベクトルでアフィン変換する
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	return {
		vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0],
		vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1],
		vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2],
	};
}

// fromをtoへ向ける単位Quaternionを求める
Quaternion DirectionToDirection(const Vector3& from, const Vector3& to) {
	Vector3 unitFrom = Normalize(from);
	Vector3 unitTo = Normalize(to);
	// 零ベクトルは方向を持たないため、無回転として扱う
	if (Dot(unitFrom, unitFrom) == 0.0f || Dot(unitTo, unitTo) == 0.0f) {
		return IdentityQuaternion();
	}
	float dot = std::clamp(Dot(unitFrom, unitTo), -1.0f, 1.0f);
	if (dot <= -1.0f + kEpsilon) {
		// 真逆ではfromと平行にならない基準軸を選び、垂直な軸で180度回す
		Vector3 referenceAxis = std::abs(unitFrom.x) < std::abs(unitFrom.z)
			? Vector3{1.0f, 0.0f, 0.0f} : Vector3{0.0f, 0.0f, 1.0f};
		Vector3 axis = Normalize(Cross(unitFrom, referenceAxis));
		return {axis.x, axis.y, axis.z, 0.0f};
	}
	Vector3 cross = Cross(unitFrom, unitTo);
	return Normalize(Quaternion{cross.x, cross.y, cross.z, 1.0f + dot});
}

// 単位Quaternion同士を最短経路で球面線形補間する
Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, float t) {
	// 正規化は呼び出し側で行い、この関数内では繰り返さない
	t = std::clamp(t, 0.0f, 1.0f);
	Quaternion end = q1;
	float dot = Dot(q0, end);
	if (dot < 0.0f) {
		// 同じ回転を表す符号反転側を使い、最短経路を選ぶ
		end = -end;
		dot = -dot;
	}
	// 丸め誤差によるacosの定義域外を防ぐ
	dot = std::clamp(dot, 0.0f, 1.0f);
	if (dot > 1.0f - kEpsilon) {
		// 角度がほぼ0のときは極限の線形補間を使い、0除算を避ける
		return (1.0f - t) * q0 + t * end;
	}
	float theta = std::acos(dot);
	float sinTheta = std::sin(theta);
	float scale0 = std::sin((1.0f - t) * theta) / sinTheta;
	float scale1 = std::sin(t * theta) / sinTheta;
	return scale0 * q0 + scale1 * end;
}

// Quaternionを画面に表示する
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label) {
	Novice::ScreenPrintf(x, y, "%8.3f %8.3f %8.3f %8.3f : %s",
		quaternion.x, quaternion.y, quaternion.z, quaternion.w, label);
}

// ベクトルを画面に表示する
void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%8.3f %8.3f %8.3f : %s", vector.x, vector.y, vector.z, label);
}

// 行列を画面に表示する
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);
	for (int row = 0; row < 4; ++row) {
		Novice::ScreenPrintf(x, y + kRowHeight * (row + 1), "%8.3f %8.3f %8.3f %8.3f",
			matrix.m[row][0], matrix.m[row][1], matrix.m[row][2], matrix.m[row][3]);
	}
}

// 補間操作用UIをまとめて描画する
void DrawSlerpControls(float& t) {
#ifdef _DEBUG
	ImGui::SetNextWindowPos(ImVec2(20.0f, 420.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(430.0f, 130.0f), ImGuiCond_Once);
	if (ImGui::Begin("Slerp Controls")) {
		ImGui::SliderFloat("t", &t, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
		if (ImGui::Button("Reset")) {
			t = 0.5f;
		}
		ImGui::Text("Unit inputs / shortest path / radians");
		ImGui::Text("Identical rotations use the linear limit.");
	}
	ImGui::End();
#else
	(void)t;
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
	Quaternion rotation0 = Normalize(MakeRotateAxisAngleQuaternion({0.71f, 0.71f, 0.0f}, 0.3f));
	Quaternion rotation1 = Normalize(MakeRotateAxisAngleQuaternion({0.71f, 0.0f, 0.71f}, 3.141592f));
	const int kSampleCount = 5;
	const float sampleT[kSampleCount] = {0.0f, 0.3f, 0.5f, 0.7f, 1.0f};
	const char* labels[kSampleCount] = {"t = 0.0", "t = 0.3", "t = 0.5", "t = 0.7", "t = 1.0"};
	float t = 0.5f;

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

		// 課題の5つの補間結果を求める
		Quaternion interpolates[kSampleCount] = {};
		for (int index = 0; index < kSampleCount; ++index) {
			interpolates[index] = Slerp(rotation0, rotation1, sampleT[index]);
		}
		// 同一回転と符号反転の特異点、操作中の結果を求める
		Quaternion same = Slerp(rotation0, rotation0, 0.5f);
		Quaternion oppositeSign = Slerp(rotation0, -rotation0, 0.5f);
		Quaternion preview = Slerp(rotation0, rotation1, t);
		Matrix4x4 previewMatrix = MakeRotateMatrix(preview);
		Vector3 previewVector = RotateVector({1.0f, 0.0f, 0.0f}, preview);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 課題の入力と実行結果を表示する
		Novice::ScreenPrintf(20, 20, "MT4_01_03 : Slerp / Shortest Path (x, y, z, w)");
		QuaternionScreenPrintf(20, 50, rotation0, "rotation0");
		QuaternionScreenPrintf(20, 50 + kRowHeight, rotation1, "rotation1");
		for (int index = 0; index < kSampleCount; ++index) {
			QuaternionScreenPrintf(20, 120 + index * kRowHeight, interpolates[index], labels[index]);
		}
		QuaternionScreenPrintf(20, 275, same, "Slerp(q0, q0, 0.5)");
		QuaternionScreenPrintf(20, 275 + kRowHeight, oppositeSign, "Slerp(q0, -q0, 0.5)");
		QuaternionScreenPrintf(20, 350, preview, "interactive result");
		Novice::ScreenPrintf(20, 350 + kRowHeight, "t = %.3f / Norm = %.6f", t, Norm(preview));
		MatrixScreenPrintf(650, 50, previewMatrix, "interactive rotation matrix");
		VectorScreenPrintf(650, 210, previewVector, "rotated +X");
		DrawSlerpControls(t);

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
