package com.app.threedify.ui.gallery

import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.icu.text.SimpleDateFormat
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Environment
import android.provider.Settings
import android.text.Editable
import android.text.TextWatcher
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.RequiresApi
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import androidx.recyclerview.widget.GridLayoutManager
import com.app.threedify.data.filesystem.FileSystemObjScanner
import com.app.threedify.databinding.FragmentGalleryBinding
import com.app.threedify.manager.ObjFileSearchManager
import com.app.threedify.ui.gallery.helpers.ObjFile
import com.app.threedify.ui.gallery.helpers.ObjFileAdapter
import org.the3deer.app.model3D.view.ModelActivity
import java.io.File
import java.nio.file.Path
import java.util.Date
import java.util.Locale

class GalleryFragment<LinearLayout> : Fragment() {
    private var _binding: FragmentGalleryBinding? = null
    private val binding get() = _binding!!
    private lateinit var adapter: ObjFileAdapter
    private lateinit var sharedPreferences : SharedPreferences // for storing settings
    private lateinit var folderSpinnerAdapter: ArrayAdapter<String>//options for dd
    @RequiresApi(Build.VERSION_CODES.R)
    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.StartActivityForResult()) {
            if (hasStoragePermission()) {
                setupUI()
            } else {
                Toast.makeText(
                    requireContext(), "Permission denied. Please, give storage permission to app in settings.",
                    Toast.LENGTH_LONG
                ).show()
            }
        }

    // Opens the app's settings page to allow the user manually give permission
    @RequiresApi(Build.VERSION_CODES.R)
    private fun openAppSettings() {
        val intent = Intent(Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION)
        intent.data = Uri.fromParts("package", requireActivity().packageName, null)
        startActivity(intent)
    }

    @RequiresApi(Build.VERSION_CODES.R)
    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View {
        _binding = FragmentGalleryBinding.inflate(inflater, container, false)

        sharedPreferences = requireContext().getSharedPreferences("gallery_preferences", Context.MODE_PRIVATE)

        val root: View = binding.root

        if (hasStoragePermission()) {
            setupUI()
        } else {
            openAppSettings()
        }

        return root
    }


    @RequiresApi(Build.VERSION_CODES.R)
    override fun onResume() {
        super.onResume()
        if (hasStoragePermission()) {
            setupUI()
        } else {
            Toast.makeText(
                requireContext(), "Storage permission is still needed to run this application",
                Toast.LENGTH_LONG
            ).show()
            openAppSettings()
        }
    }

    // Check if the app has permission to manage all files on the device
    @RequiresApi(Build.VERSION_CODES.R)
    private fun hasStoragePermission(): Boolean {
        return Environment.isExternalStorageManager()
    }


    private fun setupUI() {

        val galleryViewModel = ViewModelProvider(this)[GalleryViewModel::class.java]

        ///////////In this lambda, you can specify what you want when you press (where I make the toastik.)
        adapter = ObjFileAdapter(emptyList()) { selectedFile ->
            Toast.makeText(requireContext(), " Chosen file: ${selectedFile.name}", Toast.LENGTH_SHORT).show()
            initialize3DViewer(selectedFile.path)
            //alternative initializing variant
            //ObjectViewer.view(requireContext(), File(selectedFile.path))
        }
        binding.recyclerView.adapter = adapter
        binding.recyclerView.layoutManager = GridLayoutManager(requireContext(), 2) // Two items per row

        galleryViewModel.objFiles.observe(viewLifecycleOwner) { files ->
            adapter.updateList(files)
        }

        // Populate the dd with folder options
        val objFileScanner = FileSystemObjScanner()
        val fileSearchManager = ObjFileSearchManager(objFileScanner)
        val subDirs = fileSearchManager.getSubDir(File("/storage/emulated/0/"))
        val folderNames = mutableListOf<String>()
        subDirs.forEach { file ->
            folderNames.add(file.name)
        }
        folderSpinnerAdapter = ArrayAdapter(requireContext(), android.R.layout.simple_spinner_item, folderNames)
        folderSpinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.folderSpinner.adapter = folderSpinnerAdapter

        val defaultFolder = "Download"
        val defaultFolderIndex = folderNames.indexOf(defaultFolder)

        if (defaultFolderIndex != -1) {
            binding.folderSpinner.setSelection(defaultFolderIndex)
            scanObjFiles(File("/storage/emulated/0/$defaultFolder"))
        } else {
            scanObjFiles(File("/storage/emulated/0/"))
        }
        // Handle item selection in the Spinner
        binding.folderSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener{
            override fun onItemSelected(parent: AdapterView<*>?, view: View?, position: Int, id: Long) {
                val selectedFolder = subDirs[position]
                editScannerPath(selectedFolder)
                scanObjFiles(selectedFolder)
            }

            override fun onNothingSelected(parent: AdapterView<*>?) {
                // Handle the case where no folder is selected, if needed
            }
        }
        binding.searchBar.addTextChangedListener(object : TextWatcher {
            override fun afterTextChanged(s: Editable?) {
                val filteredList = galleryViewModel.objFiles.value?.filter {
                    it.name.contains(s.toString(), ignoreCase = true)
                } ?: emptyList()
                adapter.updateList(filteredList)
            }

            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}

            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
        })

        scanObjFiles(File("/storage/emulated/0/Download"))
    }

    private fun scanObjFiles(file: File) {
        val galleryViewModel = ViewModelProvider(this)[GalleryViewModel::class.java]
        val objFileScanner = FileSystemObjScanner()
        val fileSearchManager = ObjFileSearchManager(objFileScanner)
        val subDirs = fileSearchManager.getSubDir(File("/storage/emulated/0"))
        val directories = listOf(
            file
        )
        val objFiles = fileSearchManager.findObjFiles(directories).map {
            val size = getFileSize(it)
            val date = getFileCreationDate(it)
            ObjFile(it.name, it.path, size, date)
        }
        galleryViewModel.updateObjFiles(objFiles)
    }


    private fun editScannerPath(newPath: File){
        val editor = sharedPreferences.edit()
        editor.putString("searchPath", newPath.path)
        editor.apply()
    }


    private fun getScannerPath(): File {
        return File(sharedPreferences.getString("searchPath", "/storage/emulated/0")?: "/storage/emulated/0")
    }

    fun getFileSize(file: File): String {
        val sizeInBytes = file.length()
        val sizeInKB = sizeInBytes / 1024
        val sizeInMB = sizeInKB / 1024
        return if (sizeInMB > 0) {
            "$sizeInMB MB"
        } else {
            "$sizeInKB KB"
        }
    }
    private fun initialize3DViewer(filePath: String) {
        val fileUri = Uri.fromFile(File(filePath))

        val intent = Intent(requireContext(), ModelActivity::class.java)
        intent.putExtra("uri", fileUri.toString())
        intent.putExtra("type", "-1")
        intent.putExtra("immersiveMode", "false")
        intent.putExtra("backgroundColor", "1.0 1.0 1.0 1.0")
        startActivity(intent)
    }

    fun getFileCreationDate(file: File): String {
        val lastModified = file.lastModified()
        val date = Date(lastModified)
        val dateFormat = SimpleDateFormat("dd MMMM, HH:mm", Locale.getDefault())
        return dateFormat.format(date)
    }
    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
