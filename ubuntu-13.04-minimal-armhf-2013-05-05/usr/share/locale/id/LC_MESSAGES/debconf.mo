��    H      \  a   �         o   !  ?   �  �   �  .   h  #   �     �  '   �     �          '     ;  (   J     s  K   �     �     �     �  -   �     -	     <	  R   D	     �	     �	  8   �	  M   �	  k   C
  8   �
  (   �
            u   5     �     �  X   �  @        O     e  ;   �  6   �  7   �  �   -  /   �  4   �  =     Y   Y  �  �  )   w  7   �     �  1   �  '   *  .   R  C   �  F  �       �   %     �     �  n   �     <  @   U     �  &   �     �     �  '   �       !   0     R  a   n     �  �  �  �   �  8     �   @  (   �  (        =  "   W  	   z     �     �     �  )   �      �  S        Z  #   b     �  A   �     �     �  j   �     \     l  :   �  K   �  �     =   �  !   �     �  %   �  t        �     �  \   �  J   �     H     Z  J   u  1   �  <   �  �   /  >   �  G   �  >   A   ]   �     �   0   �"  9   #  &   Q#  9   x#  /   �#  1   �#  H   $  �  ]$     �%  �   &     �&     �&  w   �&     -'  :   F'     �'  2   �'     �'     �'  5   �'  !   (  "   ?(     b(  k   �(     �(        5       (       C                    ?                       1      B                !   @       +                  )       4   >   -   <                 9   0   =   7               .      F   /   D          :       2                            ,   3   $          E   H         ;      &      8          "       %   #   *   
      G      	   6   A   '    
        --outdated		Merge in even outdated translations.
	--drop-old-templates	Drop entire outdated templates. 
  -o,  --owner=package		Set the package that owns the command.   -f,  --frontend		Specify debconf frontend to use.
  -p,  --priority		Specify minimum priority question to show.
       --terse			Enable terse mode.
 %s failed to preconfigure, with exit status %s %s is broken or not fully installed %s is fuzzy at byte %s: %s %s is fuzzy at byte %s: %s; dropping it %s is missing %s is missing; dropping %s %s is not installed %s is outdated %s is outdated; dropping whole template! %s must be run as root (Enter zero or more items separated by a comma followed by a space (', ').) Back Cannot read status file: %s Choices Config database not specified in config file. Configuring %s Debconf Debconf is not confident this error message was displayed, so it mailed it to you. Debconf on %s Debconf, running at %s Dialog frontend is incompatible with emacs shell buffers Dialog frontend requires a screen at least 13 lines tall and 31 columns wide. Dialog frontend will not work on a dumb terminal, an emacs shell buffer, or without a controlling terminal. Enter the items you want to select, separated by spaces. Extracting templates from packages: %d%% Help Ignoring invalid priority "%s" Input value, "%s" not found in C choices! This should never happen. Perhaps the templates were incorrectly localized. More Next No usable dialog-like program is installed, so the dialog based frontend cannot be used. Note: Debconf is running in web mode. Go to http://localhost:%i/ Package configuration Preconfiguring packages ...
 Problem setting up the database defined by stanza %s of %s. TERM is not set, so the dialog frontend is not usable. Template #%s in %s does not contain a 'Template:' line
 Template #%s in %s has a duplicate field "%s" with new value "%s". Probably two templates are not properly separated by a lone newline.
 Template database not specified in config file. Template parse error near `%s', in stanza #%s of %s
 Term::ReadLine::GNU is incompatable with emacs shell buffers. The Sigils and Smileys options in the config file are no longer used. Please remove them. The editor-based debconf frontend presents you with one or more text files to edit. This is one such text file. If you are familiar with standard unix configuration files, this file will look familiar to you -- it contains comments interspersed with configuration items. Edit the file, changing any items as necessary, and then save it and exit. At that point, debconf will read the edited file, and use the values you entered to configure the system. This frontend requires a controlling tty. Unable to load Debconf::Element::%s. Failed because: %s Unable to start a frontend: %s Unknown template field '%s', in stanza #%s of %s
 Usage: debconf [options] command [args] Usage: debconf-communicate [options] [package] Usage: debconf-mergetemplate [options] [templates.ll ...] templates Usage: dpkg-reconfigure [options] packages
  -a,  --all			Reconfigure all packages.
  -u,  --unseen-only		Show only not yet seen questions.
       --default-priority	Use default priority instead of low.
       --force			Force reconfiguration of broken packages.
       --no-reload		Do not reload templates. (Use with caution.) Valid priorities are: %s You are using the editor-based debconf frontend to configure your system. See the end of this document for detailed instructions. _Help apt-extracttemplates failed: %s debconf-mergetemplate: This utility is deprecated. You should switch to using po-debconf's po2debconf program. debconf: can't chmod: %s delaying package configuration, since apt-utils is not installed falling back to frontend: %s must specify some debs to preconfigure no none of the above please specify a package to reconfigure template parse error: %s unable to initialize frontend: %s unable to re-open stdin: %s warning: possible database corruption. Will attempt to repair by adding back missing question %s. yes Project-Id-Version: debconf 1.5.45
Report-Msgid-Bugs-To: 
POT-Creation-Date: 2013-01-08 10:11+0000
PO-Revision-Date: 2012-07-29 23:56+0700
Last-Translator: Andika Triwidada <andika@gmail.com>
Language-Team: Debian Indonesia L10N Team <debid@yahoogroups.com>
Language: id
MIME-Version: 1.0
Content-Type: text/plain; charset=ISO-8859-1
Content-Transfer-Encoding: 8bit
X-Poedit-Language: Bahasa Indonesia
X-Poedit-Country: Indonesia
 
        --outdated		Gabungkan bahkan untuk terjemahan lama (outdated).
	--drop-old-templates	Buang semua template lama (outdated). 
  -o,  --owner=paket		Set paket yang memiliki perintah.   -f,  --frontend		Tentukan antar muka debconf yg dipakai.
  -p,  --priority		Tentukan prioritas pertanyaan minimum yg ditampilkan.
       --terse			Mampukan mode terse.
 Prakonfigurasi %s gagal dengan status %s %s rusak atau tidak sepenuhnya terpasang %s fuzzy pada byte %s: %s %s fuzzy pada byte %s: %s; dihapus %s hilang %s hilang; %s dihapus %s tidak dipasang %s sudah usang %s sudah usang; seluruh template dihapus! %s harus dijalankan sebagai root (Masukkan nol item atau lebih, dipisahkan dengan koma diikuti dengan spasi (', ').) Kembali Tak dapat membaca berkas status: %s Pilihan Basis data konfigurasi tidak ditentukan dalam berkas konfigurasi. Sedang mengonfigurasi %s Debconf Debconf tidak yakin bahwa pesan kesalahan ini ditampilkan, sehingga mengirimnya melalui surel kepada Anda. Debconf pada %s Debconf, berjalan pada %s Antarmuka dialog tidak sesuai dengan penyangga shell emacs Antarmuka dialog membutuhkan layar minimal berukuran 13 baris dan 31 kolom. Antar muka dialog tidak akan bekerja pada sebuah terminal dumb, sebuah penyangga shell emacs, atau tanpa sebuah terminal pengendali Masukkan item yang ingin Anda pilih, dipisahkan dengan spasi. Ekstrak template dari paket: %d%% Panduan Abaikan prioritas "%s" yang tidak sah Nilai masukan, "%s" tidak ditemukan dalam pilihan C! Hal ini tidak boleh terjadi. Mungkin lokalisasi template salah. Lagi Lanjut Tidak ada program dialog yang dapat dipakai. Antarmuka berbasis dialog tidak dapat digunakan Catatan: Debconf berjalan dalam modus web. Cobalah ke http://localhost:%i/ Konfigurasi paket Prakonfigurasi paket ... 
 Ada masalah saat menyetel basis data yang ditentukan oleh bait %s dari %s. TERM tidak diset, antarmuka dialog tidak berguna. Template #%s dalam %s tidak berisi sebuah baris 'Template:'
 Template #%s dalam %s memiliki ruas ganda "%s" dengan nilai "%s". Kemungkinan kedua template tidak dipisahkan oleh baris baru yang benar.
 Basis data template belum ditentukan dalam berkas konfigurasi. Ada kesalahan penguraian template sekitar '%s', dalam bait #%s dari %s
 Term::ReadLine::GNU tidak sesuai dengan penyangga shell emacs. Pilihan Sigils dan Smileys dalam berkas konfigurasi tidak lagi digunakan. Silakan hapus saja. Antarmuka debconf berbasis editor menyajikan kepada Anda satu atau lebih berkas teks yang harus diubah. Ini termasuk salah satu berkas tersebut. Jika Anda tidak terbiasa dengan berkas konfigurasi standar unix, berkas ini akan tampak mudah bagi Anda -- berkas ini berisi keterangan yang diikuti dengan item konfigurasi. Sunting file ini, ganti item yang diperlukan, dan simpan dan keluar. Pada titik ini debconf akan membaca berkas yang telah diubah, dan menggunakan nilai yang Anda masukkan untuk mengonfigurasi sistem. Antarmuka ini membutuhkan sebuah tty pengendali. Tidak dapat memuat Debconf::Element::%s. Gagal karena: %s Tak dapat memulai sebuah antarmuka: %s Ruas template '%s' tidak dikenal, dalam bait #%s dari %s
 Pengunaan: debconf [pilihan] perintah [argumen] Penggunaan: debconf-communicate [pilihan] [paket] Penggunaan: debconf-mergetemplate [pilihan] [templates.ll ...] templates Pengunaan: dpkg-reconfigure [pilihan] paket-paket
  -a,  --all			Konfigurasi kembali semua paket.
  -u,  --unseen-only		Hanya tampilkan pertanyaan yg belum dilihat.
       --default-priority	Gunakan prioritas bawaan, bukan yang rendah.
       --force			Paksakan konfigurasi kembali paket-paket yang rusak.
       --no-reload		Jangan muat kembali templet-templet. (Gunakan secara hati-hati) Prioritas yang sah adalah: %s Anda sedang menggunakan antarmuka debconf berbasis editor untuk mengonfigurasi sistem. Lihat pada akhir dokumen ini untuk petunjuk lengkap. _Panduan apt-extracttemplates gagal: %s debconf-mergetemplate: Utilitasi ini telah ditinggalkan. Anda sebaiknya menggunakan program po2debconf dari po-debconf. debconf: chmod gagal: %s menunda konfigurasi paket karena apt-utils tidak terpasang kembali ke antarmuka: %s harus menentukan beberapa deb untuk prakonfigurasi tidak tak satupun di atas mohon tentukan sebuah paket untuk dikonfigurasi ulang kesalahan penguraian template: %s tak dapat menyiapkan antarmuka: %s tidak dapat membuka stdin: %s Peringatan: kemungkinan basis data rusak. Akan diperbaiki dengan menanyakan kembali hal-hal yang hilang %s. ya 