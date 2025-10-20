import clr
import os
import sys

# --- 設定 ---
# スクリプト(.py)または実行可能ファイル(.exe)と同じディレクトリにある
# "LibreHardwareMonitorLib.dll" を自動的に探します。
try:
    # PyInstallerなどで固めた.exeの場合に対応
    base_path = sys._MEIPASS
except AttributeError:
    # 通常の.pyファイルとして実行した場合
    base_path = os.path.dirname(os.path.abspath(__file__))

dll_path = os.path.join(base_path, "LibreHardwareMonitorLib.dll")
# --- 設定ここまで ---

try:
    if not os.path.exists(dll_path):
        print(f"エラー: '{dll_path}' が見つかりません。")
        print("このスクリプトと同じフォルダに LibreHardwareMonitorLib.dll を配置してください。")
        input("Enterキーを押して終了します...")
        exit()

    clr.AddReference(dll_path)
    from LibreHardwareMonitor import Hardware

except Exception as e:
    print(f"DLLのロード中にエラーが発生しました: {e}")
    input("Enterキーを押して終了します...")
    exit()


def print_cpu_temp_sensor_names():
    """CPUの温度センサーの名前をすべてコンソールに出力する"""
    computer = None
    try:
        computer = Hardware.Computer()
        computer.IsCpuEnabled = True
        computer.Open()

        print("--- CPU温度センサー名リスト ---")
        found_cpu = False

        for hw in computer.Hardware:
            if hw.HardwareType == Hardware.HardwareType.Cpu:
                found_cpu = True
                hw.Update()  # 最新の情報を取得

                # センサーを名前でソートして表示を安定させる
                sorted_sensors = sorted(hw.Sensors, key=lambda s: s.Name)

                for sensor in sorted_sensors:
                    if sensor.SensorType == Hardware.SensorType.Temperature:
                        # センサー名と現在の値を出力
                        value_str = f"{sensor.Value:.2f}°C" if sensor.Value is not None else "N/A"
                        print(f"名前: '{sensor.Name}' | 現在の値: {value_str}")

        if not found_cpu:
            print("CPUが見つかりませんでした。")

        print("------------------------------")

    except Exception as e:
        print(f"センサー情報の取得中にエラーが発生しました: {e}")
    finally:
        if computer:
            computer.Close()


if __name__ == "__main__":
    print_cpu_temp_sensor_names()
    input("\n調査完了。Enterキーを押して終了します...")

