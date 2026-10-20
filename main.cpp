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

// 入力用UIをまとめて描画する
void DrawRotationControls(Vector3& axis, float& angle, Vector3& point) {
#ifdef _DEBUG
	ImGui::SetNextWindowPos(ImVec2(20.0f, 370.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(500.0f, 175.0f), ImGuiCond_Once);
	if (ImGui::Begin("Rotation Controls")) {
		ImGui::DragFloat3("Axis", &axis.x, 0.01f);
		ImGui::DragFloat("Angle (rad)", &angle, 0.01f);
		ImGui::DragFloat3("Point", &point.x, 0.01f);
		if (ImGui::Button("Reset")) {
			axis = {1.0f, 0.4f, -0.2f};
			angle = 0.45f;
			point = {2.1f, -0.9f, 1.3f};
		}
		ImGui::Text("Row vectors: v * R / zero axis: identity fallback.");
	}
	ImGui::End();
#else
	(void)axis;
	(void)angle;
	(void)point;
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
	Vector3 axis = {1.0f, 0.4f, -0.2f};
	float angle = 0.45f;
	Vector3 pointY = {2.1f, -0.9f, 1.3f};

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

		// 同じ回転をQuaternionと行列で求める
		Quaternion rotation = MakeRotateAxisAngleQuaternion(axis, angle);
		Matrix4x4 rotateMatrix = MakeRotateMatrix(rotation);
		Vector3 rotateByQuaternion = RotateVector(pointY, rotation);
		Vector3 rotateByMatrix = Transform(pointY, rotateMatrix);
		float error = Length(rotateByQuaternion - rotateByMatrix);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 課題の入力と実行結果を表示する
		Novice::ScreenPrintf(20, 20, "MT4_01_01 : Quaternion Rotation / Row-vector Matrix");
		QuaternionScreenPrintf(20, 50, rotation, "rotation (x,y,z,w)");
		MatrixScreenPrintf(20, 80, rotateMatrix, "rotateMatrix");
		VectorScreenPrintf(20, 80 + kRowHeight * 6, rotateByQuaternion, "rotateByQuaternion");
		VectorScreenPrintf(20, 80 + kRowHeight * 7, rotateByMatrix, "rotateByMatrix");
		Novice::ScreenPrintf(20, 80 + kRowHeight * 8, "Difference = %.8f / Norm(rotation) = %.6f", error, Norm(rotation));
		DrawRotationControls(axis, angle, pointY);

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
