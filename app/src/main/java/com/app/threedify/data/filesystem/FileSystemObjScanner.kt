package com.app.threedify.data.filesystem

import com.app.threedify.domain.objFileScanner
import java.io.File

class FileSystemObjScanner : objFileScanner {
    override fun scanDirectories(directories: List<File>): List<File> {
        val objFiles = mutableListOf<File>();
        directories.forEach { directory ->
            if (directory.isDirectory) {
                directory.walkTopDown().forEach { file ->
                    if (file.extension.equals("obj", ignoreCase = true)) {
                        objFiles.add(file)
                    }
                }
            }
        }
        return objFiles;
    }
}

