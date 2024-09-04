package com.app.threedify.manager

import com.app.threedify.domain.IObjFileScanner
import java.io.File

class ObjFileSearchManager(private val IObjFileScanner: IObjFileScanner) {
    fun findObjFiles(directories: List<File>):List<File>{
        return IObjFileScanner.scanDirectories(directories);
    }
}