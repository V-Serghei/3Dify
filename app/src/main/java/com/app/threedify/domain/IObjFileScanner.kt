package com.app.threedify.domain

import java.io.File

interface IObjFileScanner {
    fun  scanDirectories(directories: List<File>): List<File>
    fun getSubDirectories(targetDirectory: File) : List<File>
}