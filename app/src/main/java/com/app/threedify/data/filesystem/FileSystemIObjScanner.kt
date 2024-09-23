package com.app.threedify.data.filesystem

import com.app.threedify.domain.IObjFileScanner
import java.io.File

class FileSystemIObjScanner : IObjFileScanner {
    override fun scanDirectories(directories: List<File>): List<File> {
        val objFiles = mutableListOf<File>();
        directories.forEach { directory ->
            if (directory.isDirectory) {
                directory.walkTopDown().forEach { file ->
                    if (file.extension.equals("obj", ignoreCase = true)||file.extension.equals("stl", ignoreCase = true)
                        ||file.extension.equals("ply", ignoreCase = true)||file.extension.equals("off", ignoreCase = true)) {
                        objFiles.add(file)
                    }
                }
            }
        }
        return objFiles;
    }
}

