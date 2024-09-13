package com.app.threedify.data.filesystem

import com.app.threedify.domain.IObjFileScanner
import java.io.File

class FileSystemObjScanner : IObjFileScanner {
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
    override fun getSubDirectories(targetDirectory: File) :List<File>{
        val directories = mutableListOf<File>()
        targetDirectory.listFiles()?.forEach { directory ->
            if(directory.isDirectory){
                directories.add(directory)
            }
        }
        return directories
    }
}

