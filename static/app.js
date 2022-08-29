console.log("Hello from app.js")

function requestFile()
{
    const xhttp = new XMLHttpRequest();
    xhttp.onload = function() {
        if (xhttp.status == 200) {
            console.log(xhttp.response)
            console.log("yepa")           

        } else {
            console.log("Error occurred");
        }
    };
    console.log("Requesting file!")
    xhttp.open("GET", "/request_data", true);
    xhttp.send();

    //console.log(xhttp.response)
}