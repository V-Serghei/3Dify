package com.app.threedify.domain

import java.io.File

interface objFileScanner {
    fun  scanDirectories(directories: List<File>): List<File>
}