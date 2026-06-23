package com.app.threedify.ui.gallery.helpers

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.app.threedify.R

data class ObjFile(val name: String, val path: String, val size: String, val date: String, val extension: String)

class ObjFileAdapter(
    private var objFiles: List<ObjFile>,
    private val onItemClick: (ObjFile, View) -> Unit
) : RecyclerView.Adapter<ObjFileAdapter.ObjFileViewHolder>() {

    class ObjFileViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView) {
        val fileName: TextView = itemView.findViewById(R.id.file_name)
        val fileIcon: ImageView = itemView.findViewById(R.id.file_icon)
        val fileSize: TextView = itemView.findViewById(R.id.file_size)
        val fileDate: TextView = itemView.findViewById(R.id.file_date)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ObjFileViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.obj_file_item, parent, false)
        return ObjFileViewHolder(view)
    }

    override fun onBindViewHolder(holder: ObjFileViewHolder, position: Int) {
        val objFile = objFiles[position]
        holder.fileName.text = objFile.name
        when (objFile.extension) {
            "obj" -> holder.fileIcon.setImageResource(R.drawable.file_obj)
            "stl" -> holder.fileIcon.setImageResource(R.drawable.file_stl)
            "ply" -> holder.fileIcon.setImageResource(R.drawable.file_ply)
            "off" -> holder.fileIcon.setImageResource(R.drawable.file_off)
        }
        holder.fileSize.text = "Size: ${objFile.size}"
        holder.fileDate.text = objFile.date

        holder.itemView.setOnClickListener {
            onItemClick(objFile, holder.itemView)
        }
    }

    override fun getItemCount() = objFiles.size

    fun updateList(newList: List<ObjFile>) {
        objFiles = newList
        notifyDataSetChanged()
    }

}