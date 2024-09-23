import trimesh
import os

def convert_3d_model(input_file_path: str, input_type: str, output_type: str, output_folder: str):
    """
    Конвертирует 3D модель из одного формата в другой и сохраняет в указанную папку.

    :param input_file_path: Путь к исходному файлу 3D модели
    :param input_type: Исходный формат файла (например: .obj, .stl, .blend, .3ds)
    :param output_type: Формат, в который нужно конвертировать (например: .obj, .stl, .blend, .3ds)
    :param output_folder: Папка, куда сохранить сконвертированный файл
    """
    supported_formats = ['.obj', '.stl', '.blend', '.3ds']

    if input_type not in supported_formats or output_type not in supported_formats:
        raise ValueError(f"Неподдерживаемые форматы! Доступные форматы: {supported_formats}")

    try:
        mesh = trimesh.load(input_file_path)
    except Exception as e:
        raise RuntimeError(f"Ошибка загрузки файла: {e}")

    base_name = os.path.basename(input_file_path).replace(input_type, "")
    output_file_path = os.path.join(output_folder, f"{base_name}{output_type}")

    try:
        mesh.export(output_file_path)
        msg = f"Файл успешно сохранён: {output_file_path}"
        msg = f" {output_file_path}"
    except Exception as e:
        raise RuntimeError(f"Ошибка сохранения файла: {e}")

    return msg

def check_trimesh():

    cube = trimesh.creation.box()
    volume = cube.volume
    num_faces = len(cube.faces)

    return f"Trimesh is working! Volume: {volume}, Number of faces: {num_faces}"
