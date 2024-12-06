package com.app.threedify.manager

import com.app.threedify.domain.IObjFileScanner
import java.io.File

class ObjFileSearchManager(private val iObjFileScanner: IObjFileScanner) {
    fun findObjFiles(directories: List<File>):List<File>{
        return iObjFileScanner.scanDirectories(directories)
    }
    fun getSubDir(directory: File): List<File>{
        return iObjFileScanner.getSubDirectories(directory)
    }
}