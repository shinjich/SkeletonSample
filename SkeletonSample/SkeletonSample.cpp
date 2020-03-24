#ifndef STRICT
#define STRICT	// 厳密なコードを型を要求する
#endif
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#pragma comment( lib, "k4a.lib" )
#pragma comment( lib, "k4abt.lib" )

#define ENABLE_CSV_OUTPUT		1			// 1=CSV 出力を有効にする

#define MAX_BODIES				8

// アプリケーションのタイトル名
static const TCHAR szClassName[] = TEXT("骨格追跡サンプル");
HWND g_hWnd = NULL;							// アプリケーションのウィンドウ
HPEN g_hPen = NULL;							// 描画用のペン

k4a_device_t g_hAzureKinect = nullptr;		// Azure Kinect のデバイスハンドル
k4abt_tracker_t g_hTracker = nullptr;		// ボディトラッカーのハンドル
k4a_calibration_t g_Calibration;			// Azure Kinect のキャリブレーションデータ
k4abt_skeleton_t g_Skeleton[MAX_BODIES];	// ユーザーの 3D 骨格情報
k4a_float2_t g_fSkeleton2D[MAX_BODIES][K4ABT_JOINT_COUNT] = { 0.0f, };	// ユーザーの 2D 骨格座標 (表示用)
uint32_t g_uBodies = 0;						// 骨格追跡されている人数

#if ENABLE_CSV_OUTPUT
HANDLE g_hFile = INVALID_HANDLE_VALUE;		// CSV ファイルハンドル
#endif

// Kinect を初期化する
k4a_result_t CreateKinect()
{
	k4a_result_t hr;

	// Azure Kinect を初期化する
	hr = k4a_device_open( K4A_DEVICE_DEFAULT, &g_hAzureKinect );
	if ( hr == K4A_RESULT_SUCCEEDED )
	{
		// カメラを開始する
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
		config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.synchronized_images_only = false;
		config.depth_delay_off_color_usec = 0;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		config.subordinate_delay_off_master_usec = 0;
		config.disable_streaming_indicator = false;

		// Azure Kinect を開始する
		hr = k4a_device_start_cameras( g_hAzureKinect, &config );
		if ( hr == K4A_RESULT_SUCCEEDED )
		{
			// キャリブレーションデータを取得する
			hr = k4a_device_get_calibration( g_hAzureKinect, config.depth_mode, config.color_resolution, &g_Calibration );
			if ( hr == K4A_RESULT_SUCCEEDED )
			{
				// 骨格追跡を開始する
				hr = k4abt_tracker_create( &g_Calibration, K4ABT_TRACKER_CONFIG_DEFAULT, &g_hTracker );
				if ( hr == K4A_RESULT_SUCCEEDED )
				{
					return hr;
				}
				else
				{
					MessageBox( NULL, TEXT("骨格追跡を開始できませんでした"), TEXT("エラー"), MB_OK );
				}
			}
			else
			{
				MessageBox( NULL, TEXT("キャリブレーション情報を取得できませんでした"), TEXT("エラー"), MB_OK );
			}
			// Azure Kinect を停止する
			k4a_device_stop_cameras( g_hAzureKinect );
		}
		else
		{
			MessageBox( NULL, TEXT("Azure Kinect を開始できませんでした"), TEXT("エラー"), MB_OK );
		}
		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
	}
	else
	{
		MessageBox( NULL, TEXT("Azure Kinect の初期化に失敗 - カメラの状態を確認してください"), TEXT("エラー"), MB_OK );
	}
	return hr;
}

// Kinect を終了する
void DestroyKinect()
{
	// 骨格追跡を無効にする
	if ( g_hTracker )
	{
		k4abt_tracker_destroy( g_hTracker );
		g_hTracker = nullptr;
	}

	if ( g_hAzureKinect )
	{
		// Azure Kinect を停止する
		k4a_device_stop_cameras( g_hAzureKinect );

		// Azure Kinect の使用をやめる
		k4a_device_close( g_hAzureKinect );
		g_hAzureKinect = nullptr;
	}
}

// KINECT のメインループ処理
uint32_t KinectProc()
{
	k4a_wait_result_t hr;
	uint32_t uBodies = 0;

	k4a_capture_t hCapture = nullptr;
	// カメラでキャプチャーする
	hr = k4a_device_get_capture( g_hAzureKinect, &hCapture, K4A_WAIT_INFINITE );
	if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
	{
		// 骨格追跡をキューする
		hr = k4abt_tracker_enqueue_capture( g_hTracker, hCapture, K4A_WAIT_INFINITE );
		// カメラキャプチャーを開放する
		k4a_capture_release( hCapture );
		if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
		{
			// 骨格追跡の結果を取得する
			k4abt_frame_t hBodyFrame = nullptr;
			hr = k4abt_tracker_pop_result( g_hTracker, &hBodyFrame, K4A_WAIT_INFINITE );
			if ( hr == K4A_WAIT_RESULT_SUCCEEDED )
			{
				// 認識された人数を取得する
				uBodies = k4abt_frame_get_num_bodies( hBodyFrame );
				if ( uBodies > MAX_BODIES )
					uBodies = MAX_BODIES;
				for( uint32_t uBody = 0; uBody < uBodies; uBody++ )
				{
					// 各人の骨格情報を取得する
					if ( k4abt_frame_get_body_skeleton( hBodyFrame, uBody, &g_Skeleton[uBody] ) == K4A_RESULT_SUCCEEDED )
					{
						for( int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++ )
						{
							int iValid = 0;
							// 3D の骨格座標を 2D スクリーン座標に変換する
							k4a_calibration_3d_to_2d( &g_Calibration, &g_Skeleton[uBody].joints[iJoint].position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &g_fSkeleton2D[uBody][iJoint], &iValid );
							if ( iValid == 0 )
							{
								// 無効な値は (0,0) に設定
								g_fSkeleton2D[uBody][iJoint].xy.x = g_fSkeleton2D[uBody][iJoint].xy.y = 0.0f;
							}
						}
					}
				}
				// 骨格追跡フレームを開放する
				k4abt_frame_release( hBodyFrame );
				g_uBodies = uBodies;
			}
		}
	}
	return uBodies;
}

#if ENABLE_CSV_OUTPUT
// CSV ファイルにデータを出力
void WriteCSV()
{
	if ( g_hFile != INVALID_HANDLE_VALUE )
	{
		char szText[2048] = "";

		// 骨格座標を 32 箇所すべて列に出力
		for( int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++ )
		{
			sprintf_s( szText, 2048, "%s,%f", szText, g_fSkeleton2D[0][iJoint].xy.y );
		}
		strcat_s( szText, 2048, "\r\n" );

		// 改行してファイル出力
		DWORD dwWritten;
		WriteFile( g_hFile, szText, (DWORD) strlen( szText ), &dwWritten, NULL );
	}
}
#endif

LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch( uMsg )
	{
	case WM_PAINT:
		{
			// 画面表示
			TCHAR szText[128];
			PAINTSTRUCT ps;
			HDC hDC = BeginPaint( hWnd, &ps );

			// 画面サイズを取得する
			RECT rect;
			GetClientRect( hWnd, &rect );

			// 深度映像の縦横幅を取得して画面サイズとの割合を求める
			const DWORD dwWidth = 640;
			const DWORD dwHeight = 576;
			const FLOAT fRelativeWidth = (FLOAT) rect.right / (FLOAT) dwWidth;
			const FLOAT fRelativeHeight = (FLOAT) rect.bottom / (FLOAT) dwHeight;

			// ペンを選択
			HPEN hPenPrev = (HPEN) SelectObject( hDC, g_hPen );

			// 文字の背景を透明に
			SetBkMode( hDC, TRANSPARENT );

			for( uint32_t uBody = 0; uBody < g_uBodies; uBody++ )
			{
				// 骨格を画面サイズに合わせてスケーリング
				LONG lSkeletonX[K4ABT_JOINT_COUNT];
				LONG lSkeletonY[K4ABT_JOINT_COUNT];
				for( int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++ )
				{
					lSkeletonX[iJoint] = (LONG) (g_fSkeleton2D[uBody][iJoint].xy.x * fRelativeWidth);
					lSkeletonY[iJoint] = (LONG) (g_fSkeleton2D[uBody][iJoint].xy.y * fRelativeHeight);

					// 座標を画面上に表示
					_stprintf_s( szText, 128, TEXT("%.2f %.2f %.2f"), g_Skeleton[uBody].joints[iJoint].position.xyz.x, g_Skeleton[uBody].joints[iJoint].position.xyz.y, g_Skeleton[uBody].joints[iJoint].position.xyz.z );
					switch( g_Skeleton[uBody].joints[iJoint].confidence_level )
					{
					case K4ABT_JOINT_CONFIDENCE_NONE:
						SetTextColor( hDC, RGB( 0, 0, 255 ) );
						break;
					case K4ABT_JOINT_CONFIDENCE_LOW:
						SetTextColor( hDC, RGB( 255, 0, 0 ) );
						break;
					case K4ABT_JOINT_CONFIDENCE_MEDIUM:
						SetTextColor( hDC, RGB( 255, 255, 0 ) );
						break;
					case K4ABT_JOINT_CONFIDENCE_HIGH:
						SetTextColor( hDC, RGB( 255, 255, 255 ) );
						break;
					default:
						SetTextColor( hDC, RGB( 128, 128, 128 ) );
						break;
					}
					TextOut( hDC, lSkeletonX[iJoint], lSkeletonY[iJoint], szText, (int) _tcslen( szText ) );
				}
				
				// 骨格の表示

				// 顔を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_EAR_LEFT], lSkeletonY[K4ABT_JOINT_EAR_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EYE_LEFT], lSkeletonY[K4ABT_JOINT_EYE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NOSE], lSkeletonY[K4ABT_JOINT_NOSE] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EYE_RIGHT], lSkeletonY[K4ABT_JOINT_EYE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_EAR_RIGHT], lSkeletonY[K4ABT_JOINT_EAR_RIGHT] );

				// 体を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_PELVIS], lSkeletonY[K4ABT_JOINT_PELVIS], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_NAVEL], lSkeletonY[K4ABT_JOINT_SPINE_NAVEL] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_CHEST], lSkeletonY[K4ABT_JOINT_SPINE_CHEST] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NECK], lSkeletonY[K4ABT_JOINT_NECK] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HEAD], lSkeletonY[K4ABT_JOINT_HEAD] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_NOSE], lSkeletonY[K4ABT_JOINT_NOSE] );

				// 腕を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HAND_LEFT], lSkeletonY[K4ABT_JOINT_HAND_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_WRIST_LEFT], lSkeletonY[K4ABT_JOINT_WRIST_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ELBOW_LEFT], lSkeletonY[K4ABT_JOINT_ELBOW_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SHOULDER_LEFT], lSkeletonY[K4ABT_JOINT_SHOULDER_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_CLAVICLE_LEFT], lSkeletonY[K4ABT_JOINT_CLAVICLE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SPINE_CHEST], lSkeletonY[K4ABT_JOINT_SPINE_CHEST] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_CLAVICLE_RIGHT], lSkeletonY[K4ABT_JOINT_CLAVICLE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_SHOULDER_RIGHT], lSkeletonY[K4ABT_JOINT_SHOULDER_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ELBOW_RIGHT], lSkeletonY[K4ABT_JOINT_ELBOW_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_WRIST_RIGHT], lSkeletonY[K4ABT_JOINT_WRIST_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_RIGHT], lSkeletonY[K4ABT_JOINT_HAND_RIGHT] );

				// 左手を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HANDTIP_LEFT], lSkeletonY[K4ABT_JOINT_HANDTIP_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_LEFT], lSkeletonY[K4ABT_JOINT_HAND_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_THUMB_LEFT], lSkeletonY[K4ABT_JOINT_THUMB_LEFT] );

				// 右手を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_HANDTIP_RIGHT], lSkeletonY[K4ABT_JOINT_HANDTIP_RIGHT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HAND_RIGHT], lSkeletonY[K4ABT_JOINT_HAND_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_THUMB_RIGHT], lSkeletonY[K4ABT_JOINT_THUMB_RIGHT] );

				// 脚を線で結ぶ
				MoveToEx( hDC, lSkeletonX[K4ABT_JOINT_FOOT_LEFT], lSkeletonY[K4ABT_JOINT_FOOT_LEFT], NULL );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ANKLE_LEFT], lSkeletonY[K4ABT_JOINT_ANKLE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_KNEE_LEFT], lSkeletonY[K4ABT_JOINT_KNEE_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HIP_LEFT], lSkeletonY[K4ABT_JOINT_HIP_LEFT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_PELVIS], lSkeletonY[K4ABT_JOINT_PELVIS] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_HIP_RIGHT], lSkeletonY[K4ABT_JOINT_HIP_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_KNEE_RIGHT], lSkeletonY[K4ABT_JOINT_KNEE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_ANKLE_RIGHT], lSkeletonY[K4ABT_JOINT_ANKLE_RIGHT] );
				LineTo( hDC, lSkeletonX[K4ABT_JOINT_FOOT_RIGHT], lSkeletonY[K4ABT_JOINT_FOOT_RIGHT] );
			}
			// ペンを元に戻す
			SelectObject( hDC, hPenPrev );
			EndPaint( hWnd, &ps );
		}
		return 0;
	case WM_CLOSE:
		DestroyWindow( hWnd );
	case WM_DESTROY:
		PostQuitMessage( 0 );
		break;
	default:
		return DefWindowProc( hWnd, uMsg, wParam, lParam );
	}
	return 0;
}

// アプリケーションの初期化 (ウィンドウや描画用のペンを作成)
HRESULT InitApp( HINSTANCE hInst, int nCmdShow )
{
	WNDCLASSEX wc = { 0, };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInst;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH) GetStockObject( BLACK_BRUSH );
	wc.lpszClassName = szClassName;
	wc.hIconSm = LoadIcon( NULL, IDI_APPLICATION );
	if ( ! RegisterClassEx( &wc ) )
	{
		MessageBox( NULL, TEXT("アプリケーションクラスの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// アプリケーションウィンドウを作成
	g_hWnd = CreateWindow( szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInst, NULL );
	if ( ! g_hWnd )
	{
		MessageBox( NULL, TEXT("ウィンドウの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	// 文字表示用のペンを作成
	g_hPen = CreatePen( PS_SOLID, 3, RGB( 0, 255, 0 ) );
	if ( ! g_hPen )
	{
		MessageBox( NULL, TEXT("ペンの初期化に失敗"), TEXT("エラー"), MB_OK );
		return E_FAIL;
	}

	ShowWindow( g_hWnd, nCmdShow );
	UpdateWindow( g_hWnd );

	return S_OK;
}

// アプリケーションの後始末
HRESULT UninitApp()
{
	// 文字表示用のペンを削除
	if ( g_hPen )
	{
		DeleteObject( (HGDIOBJ) g_hPen );
		g_hPen = NULL;
	}
	return S_OK;
}

// エントリーポイント
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, int nCmdShow )
{
	// アプリケーションを初期化する
	if ( FAILED( InitApp( hInst, nCmdShow ) ) )
		return 1;

#if ENABLE_CSV_OUTPUT
	// CSV を初期化する
	g_hFile = CreateFileA( "output.csv", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL );
#endif

	// KINECT を初期化する
	if ( FAILED( CreateKinect() ) )
		return 1;

	// アプリケーションループ
	MSG msg;
	while( GetMessage( &msg, NULL, 0, 0 ) )
	{
		// ウィンドウメッセージを処理
		TranslateMessage( &msg );
		DispatchMessage( &msg );

		// Kinect 情報に更新があれば描画
		if ( KinectProc() )
		{
#if ENABLE_CSV_OUTPUT
			// 骨格が更新されたら CSV 出力
			WriteCSV();
#endif
			// 描画
			InvalidateRect( g_hWnd, NULL, TRUE );
		}
	}

#if ENABLE_CSV_OUTPUT
	// CSV を閉じる
	if ( g_hFile != INVALID_HANDLE_VALUE )
	{
		CloseHandle( g_hFile );
		g_hFile = INVALID_HANDLE_VALUE;
	}
#endif

	// KINECT を終了する
	DestroyKinect();

	// アプリケーションを終了する
	UninitApp();

	return (int) msg.wParam;
}
