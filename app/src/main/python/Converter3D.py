import trimesh
import os

def convert_3d_model(input_file_path: str, input_type: str, output_type: str, output_folder: str):

    supported_formats = ['.obj', '.stl', '.ply', '.off']

    if input_type not in supported_formats or output_type not in supported_formats:
        return f"Неподдерживаемые форматы! Доступные форматы: {supported_formats}"

    try:
        # Пытаемся загрузить 3D модель
        mesh = trimesh.load(input_file_path)
    except Exception as e:
        return f"Ошибка загрузки файла: {e}"

    base_name = os.path.basename(input_file_path).replace(input_type, "")
    output_file_path = os.path.join(output_folder, f"{base_name}{output_type}")

    try:
        mesh.export(output_file_path)
        return f"Файл успешно сохранён: {output_file_path}"
    except Exception as e:
        return f"Ошибка сохранения файла: {e}"


def check_trimesh():

    cube = trimesh.creation.box()
    volume = cube.volume
    num_faces = len(cube.faces)

    return f"Trimesh is working! Volume: {volume}, Number of faces: {num_faces}"
