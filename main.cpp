#include <Novice.h>

#ifdef _DEBUG
#include <imgui.h>
#endif

const char kWindowTitle[] = "LC1C_14_コウケンリュウ";

// 2次元ベクトル
struct Vector2 {
	float x;
	float y;
};

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	// 円の位置と半径の初期化
	Vector2 target = {640.0f, 360.0f};
	Vector2 pos = {640.0f, 360.0f};
	const int radiusA = 12;
	const int radiusB = 20;

	// 60fpsでの追従速度
	const float deltaTime = 1.0f / 60.0f;
	float speed = 6.0f;

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
		// 追従速度を変更する
		ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(360.0f, 180.0f), ImGuiCond_Once);
		bool isWindowOpen = ImGui::Begin("Interpolation Controller");
		if (isWindowOpen) {
			ImGui::Text("Target: Mouse Position (Red Circle)");
			ImGui::SliderFloat("Speed", &speed, 0.0f, 20.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
		}
#endif

		// 円Aをマウスカーソルの位置に合わせる
		int mouseX = 0;
		int mouseY = 0;
		Novice::GetMousePosition(&mouseX, &mouseY);
		target = {static_cast<float>(mouseX), static_cast<float>(mouseY)};

		// 円Bを円Aの位置へ線形補間で近づける
		pos.x += (speed * deltaTime) * (target.x - pos.x);
		pos.y += (speed * deltaTime) * (target.y - pos.y);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

#ifdef _DEBUG
		// 追従位置と補間割合を表示する
		if (isWindowOpen) {
			ImGui::Separator();
			ImGui::Text("Mouse Pos: (%.1f, %.1f)", target.x, target.y);
			ImGui::Text("Circle Pos: (%.1f, %.1f)", pos.x, pos.y);
			ImGui::Text("Delta Time: %.4f s (60fps)", deltaTime);
			ImGui::Text("Interpolation: %.3f", speed * deltaTime);
		}
		ImGui::End();
#else
		// Releaseでは計算結果を画面に表示する
		Novice::ScreenPrintf(20, 20, "Speed: %.2f / Delta Time: %.4f s (60fps)", speed, deltaTime);
		Novice::ScreenPrintf(20, 40, "Mouse Pos: (%.1f, %.1f)", target.x, target.y);
		Novice::ScreenPrintf(20, 60, "Circle Pos: (%.1f, %.1f)", pos.x, pos.y);
#endif

		// 円Aと円Bの中心を結ぶ線を描画する
		int targetX = static_cast<int>(target.x);
		int targetY = static_cast<int>(target.y);
		int posX = static_cast<int>(pos.x);
		int posY = static_cast<int>(pos.y);
		Novice::DrawLine(posX, posY, targetX, targetY, WHITE);

		// 重なったときも両方が見えるように円Bから描画する
		Novice::DrawEllipse(posX, posY, radiusB, radiusB, 0.0f, GREEN, kFillModeSolid);
		Novice::DrawEllipse(targetX, targetY, radiusA, radiusA, 0.0f, RED, kFillModeSolid);

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
