package com.app.threedify.manager

import com.app.threedify.domain.objFileScanner
import java.io.File

class ObjFileSearchManager(private val objFileScanner: objFileScanner) {
    fun findObjFiles(directories: List<File>):List<File>{
        return objFileScanner.scanDirectories(directories);
    }
}