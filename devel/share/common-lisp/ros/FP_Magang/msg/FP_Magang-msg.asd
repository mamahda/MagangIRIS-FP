
(cl:in-package :asdf)

(defsystem "FP_Magang-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BS2PC" :depends-on ("_package_BS2PC"))
    (:file "_package_BS2PC" :depends-on ("_package"))
    (:file "PC2BS" :depends-on ("_package_PC2BS"))
    (:file "_package_PC2BS" :depends-on ("_package"))
    (:file "coordinate" :depends-on ("_package_coordinate"))
    (:file "_package_coordinate" :depends-on ("_package"))
  ))