// This Jenkinsfile defines a pipeline for Pylint validation.
// It will run on the 'pylint-vm' agent, checkout code from a public repo,
// run Pylint, and email the report.

pipeline {
    // The agent directive specifies the Jenkins agent to use.
    // 'pylint-vm-1' is the label you assigned to your virtual machine node.
    agent {
        label 'pylint-vm-1'
    }

    // The stages block contains all the steps of the pipeline.
    stages {
        // This stage checks out the code from your public GitHub repository.
        // No credentials are needed for a public repo.
        stage('Checkout') {
            steps {
                git branch: 'main', url: 'https://github.com/SSI-Labs/ros-robotmission'
            }
        }
        
        // This stage runs Pylint on all Python files and saves the output to a file.
        // The `|| true` at the end of the command ensures that the script will not fail
        // even if Pylint returns a non-zero exit code.
        stage('Pylint Analysis') {
            steps {
                sh 'pylint **/*.py > pylint_report.txt || true'
            }
        }
        
        // This stage archives the report file, making it accessible from the Jenkins UI.
        stage('Archive Artifacts') {
            steps {
                archiveArtifacts artifacts: 'pylint_report.txt', fingerprint: true
            }
        }
    }
    
    // The post block runs after all stages are complete.
    // The `always` condition ensures this block runs even if a stage fails.
    post {
        always {
            script {
                // This step sends an email with the report attached.
                // You must have the Email Extension Plugin configured in Jenkins for this to work.
                emailext (
                    subject: "Pylint Report for Build #${currentBuild.number}",
                    body: "The Pylint analysis has completed. The report is attached.",
                    attachLog: true,
                    compressLog: false,
                    to: 'hwojack@systemxi.com', // ** Replace with your email address **
                    attachmentsPattern: 'pylint_report.txt'
                )
            }
        }
    }
}
