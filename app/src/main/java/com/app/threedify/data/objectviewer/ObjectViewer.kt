package com.app.threedify.data.objectviewer

import android.content.Context
import android.content.Intent
import android.net.Uri
import com.app.threedify.domain.IObjFileViewer
import org.the3deer.app.model3D.view.ModelActivity
import java.io.File

class ObjectViewer(
    private val context: Context,
    private val file:File): IObjFileViewer {

    init {
        ///Should exist in every class which uses activities from VIEWERAPP
        System.setProperty("java.protocol.handler.pkgs", "org.the3deer.util.android")
        //URL.setURLStreamHandlerFactory(AndroidURLStreamHandlerFactory())
    }

    //Call for 3D viewer window
    override fun viewObject() {
        val fileUri: Uri = Uri.fromFile(file)

            //Creating Intent and sending it's parameters
        val intent = Intent(context, ModelActivity::class.java).apply {
            putExtra("uri", fileUri.toString())
            putExtra("type", "-1")
            putExtra("immersiveMode", "false")
            putExtra("backgroundColor", "1.0 1.0 1.0 1.0")

        }

        //Launching activity
        context.startActivity(intent)
    }
    companion object{
        fun view(context: Context, file: File){
            val viewer = ObjectViewer(context, file)
            viewer.viewObject()
        }
    }
}